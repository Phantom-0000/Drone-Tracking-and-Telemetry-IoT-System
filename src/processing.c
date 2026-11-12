#include "processing.h"

#include <math.h>
#include <string.h>

#define EARTH_METERS_PER_DEGREE_LATITUDE 110540.0
#define EARTH_METERS_PER_DEGREE_LONGITUDE 111320.0
#define STATISTICS_WINDOW_MS 30000U
#define STANDARD_GRAVITY_MPS2 9.80665f
#define DEG_TO_RAD 0.017453292519943295f

/* Start-up calibration acceptance limits. */
#define CALIBRATION_MAX_GYRO_DPS 20.0f
#define CALIBRATION_MAX_ACCEL_SPREAD_MPS2 1.0f
#define CALIBRATION_MIN_GRAVITY_MPS2 8.0f
#define CALIBRATION_MAX_GRAVITY_MPS2 11.5f
/* Give up (zero bias, standard gravity) after this many windows' time. */
#define CALIBRATION_TIMEOUT_WINDOWS 5U

/* Tilt is only estimated while |a| is close to gravity (quasi-static). */
#define TILT_MAX_ACCEL_DEV_MPS2 2.5f

static float abs_f(float x)
{
    return x < 0.0f ? -x : x;
}

static float magnitude3(float x, float y, float z)
{
    float sum = x * x + y * y + z * z;
    float guess;
    int i;

    if (!(sum > 0.0f)) {
        return 0.0f;
    }
    guess = sum > 1.0f ? sum * 0.5f : 1.0f;
    for (i = 0; i < 12; ++i) {
        guess = 0.5f * (guess + sum / guess);
    }
    return guess;
}

static float clamp_dt_seconds(uint32_t previous_ms, uint32_t current_ms)
{
    uint32_t delta_ms;

    if (current_ms <= previous_ms) {
        return 0.0f;
    }
    delta_ms = current_ms - previous_ms;
    if (delta_ms > 200U) {
        delta_ms = 200U;
    }
    return (float)delta_ms / 1000.0f;
}

static void attitude_update(struct attitude_state *state,
                            const struct imu_sample *sample)
{
    float dt;

    if (!state->initialized) {
        state->last_timestamp_ms = sample->timestamp_ms;
        state->initialized = true;
        return;
    }

    dt = clamp_dt_seconds(state->last_timestamp_ms, sample->timestamp_ms);
    state->roll_deg += sample->gx_dps * dt;
    state->pitch_deg += sample->gy_dps * dt;
    state->yaw_deg += sample->gz_dps * dt;
    state->last_timestamp_ms = sample->timestamp_ms;
}

static float wrap_angle_deg(float angle)
{
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle - 180.0f;
}

static void rotate_body_to_world(const struct attitude_state *attitude,
                                 const struct imu_sample *sample,
                                 float *world_x,
                                 float *world_y,
                                 float *world_z)
{
    const float radians = 0.017453292519943295f;
    float roll = attitude->roll_deg * radians;
    float pitch = attitude->pitch_deg * radians;
    float yaw = attitude->yaw_deg * radians;
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    *world_x = (cy * cp) * sample->ax_mps2 +
               (cy * sp * sr - sy * cr) * sample->ay_mps2 +
               (cy * sp * cr + sy * sr) * sample->az_mps2;
    *world_y = (sy * cp) * sample->ax_mps2 +
               (sy * sp * sr + cy * cr) * sample->ay_mps2 +
               (sy * sp * cr - cy * sr) * sample->az_mps2;
    *world_z = (-sp) * sample->ax_mps2 +
               (cp * sr) * sample->ay_mps2 +
               (cp * cr) * sample->az_mps2;
}

static void position_from_gps(struct position_state *position,
                              const struct telemetry_snapshot *snapshot)
{
    double latitude_scale;
    double delta_latitude;
    double delta_longitude;

    if (!snapshot->gps_fix) {
        position->last_gps_valid = false;
        return;
    }

    if (!position->origin_valid) {
        position->origin_valid = true;
        position->origin_latitude_deg = snapshot->latitude_deg;
        position->origin_longitude_deg = snapshot->longitude_deg;
        position->origin_altitude_m = snapshot->altitude_m;
        position->integration_origin_valid = true;
        position->x_m = 0.0f;
        position->y_m = 0.0f;
        position->z_m = 0.0f;
        position->vx_mps = 0.0f;
        position->vy_mps = 0.0f;
        position->vz_mps = 0.0f;
        position->last_gps_valid = false;
    }

    latitude_scale = cos(position->origin_latitude_deg * 0.017453292519943295);
    delta_latitude = snapshot->latitude_deg - position->origin_latitude_deg;
    delta_longitude = snapshot->longitude_deg - position->origin_longitude_deg;
    position->x_m = (float)(delta_longitude * EARTH_METERS_PER_DEGREE_LONGITUDE *
                            latitude_scale);
    position->y_m = (float)(delta_latitude * EARTH_METERS_PER_DEGREE_LATITUDE);
    position->z_m = snapshot->altitude_m - position->origin_altitude_m;

    if (!position->last_gps_valid) {
        position->vx_mps = 0.0f;
        position->vy_mps = 0.0f;
        position->vz_mps = 0.0f;
    } else if (snapshot->gps_timestamp_ms > position->last_gps_timestamp_ms) {
        uint32_t delta_ms =
            snapshot->gps_timestamp_ms - position->last_gps_timestamp_ms;

        if (delta_ms >= 50U && delta_ms <= 5000U) {
            float dt = (float)delta_ms / 1000.0f;
            double previous_latitude =
                position->last_gps_latitude_deg - position->origin_latitude_deg;
            double previous_longitude =
                position->last_gps_longitude_deg - position->origin_longitude_deg;
            float previous_x = (float)(previous_longitude *
                                       EARTH_METERS_PER_DEGREE_LONGITUDE *
                                       latitude_scale);
            float previous_y = (float)(previous_latitude *
                                       EARTH_METERS_PER_DEGREE_LATITUDE);
            float previous_z = position->last_gps_altitude_m -
                               position->origin_altitude_m;
            position->vx_mps = (position->x_m - previous_x) / dt;
            position->vy_mps = (position->y_m - previous_y) / dt;
            position->vz_mps = (position->z_m - previous_z) / dt;
        } else if (delta_ms > 5000U) {
            position->vx_mps = 0.0f;
            position->vy_mps = 0.0f;
            position->vz_mps = 0.0f;
        }
    }

    position->last_gps_timestamp_ms = snapshot->gps_timestamp_ms;
    position->last_gps_latitude_deg = snapshot->latitude_deg;
    position->last_gps_longitude_deg = snapshot->longitude_deg;
    position->last_gps_altitude_m = snapshot->altitude_m;
    position->last_gps_valid = true;
}

static void position_integrate_inertial(struct position_state *position,
                                        const struct attitude_state *attitude,
                                        const struct imu_sample *sample,
                                        float gravity_mps2)
{
    float dt;
    float world_x;
    float world_y;
    float world_z;
    float linear_z;

    if (!position->integration_origin_valid || attitude == NULL ||
        !attitude->initialized) {
        return;
    }

    if (!position->last_imu_valid) {
        position->last_imu_timestamp_ms = sample->timestamp_ms;
        position->last_imu_valid = true;
        return;
    }

    dt = clamp_dt_seconds(position->last_imu_timestamp_ms, sample->timestamp_ms);
    position->last_imu_timestamp_ms = sample->timestamp_ms;
    if (dt <= 0.0f) {
        return;
    }

    rotate_body_to_world(attitude, sample, &world_x, &world_y, &world_z);
    linear_z = world_z - gravity_mps2;
    position->x_m += position->vx_mps * dt + 0.5f * world_x * dt * dt;
    position->y_m += position->vy_mps * dt + 0.5f * world_y * dt * dt;
    position->z_m += position->vz_mps * dt + 0.5f * linear_z * dt * dt;
    position->vx_mps += world_x * dt;
    position->vy_mps += world_y * dt;
    position->vz_mps += linear_z * dt;
}

static void statistics_reset(struct statistics_state *statistics,
                             uint32_t timestamp_ms)
{
    /* Keep the sample sequence cursors so a new window does not re-count
     * the last environment sample published in the previous window. */
    uint32_t last_light_seq = statistics->last_light_seq;
    uint32_t last_environment_seq = statistics->last_environment_seq;

    memset(statistics, 0, sizeof(*statistics));
    statistics->window_start_ms = timestamp_ms;
    statistics->last_light_seq = last_light_seq;
    statistics->last_environment_seq = last_environment_seq;
    statistics->initialized = true;
}

static void statistics_update_channel(struct statistics_state *statistics,
                                      size_t index,
                                      float value)
{
    uint32_t count = statistics->valid_counts[index];

    statistics->sums[index] += value;
    if (count == 0U || value < statistics->minima[index]) {
        statistics->minima[index] = value;
    }
    if (count == 0U || value > statistics->maxima[index]) {
        statistics->maxima[index] = value;
    }
    statistics->valid_counts[index] = count + 1U;
}

static float statistics_mean(const struct statistics_state *statistics,
                             size_t index)
{
    uint32_t count = statistics->valid_counts[index];

    return count == 0U ? 0.0f : statistics->sums[index] / (float)count;
}

static void statistics_publish(const struct statistics_state *statistics,
                               struct telemetry_statistics *output)
{
    output->window_end_ms = statistics->window_start_ms + STATISTICS_WINDOW_MS;
    output->light_samples = statistics->valid_counts[0];
    output->environment_samples = statistics->valid_counts[1];
    output->imu_samples = statistics->valid_counts[3];
    output->light_mean = statistics_mean(statistics, 0U);
    output->light_min = statistics->minima[0];
    output->light_max = statistics->maxima[0];
    output->temperature_mean = statistics_mean(statistics, 1U);
    output->temperature_min = statistics->minima[1];
    output->temperature_max = statistics->maxima[1];
    output->humidity_mean = statistics_mean(statistics, 2U);
    output->humidity_min = statistics->minima[2];
    output->humidity_max = statistics->maxima[2];
    output->acceleration_mean = statistics_mean(statistics, 3U);
    output->acceleration_min = statistics->minima[3];
    output->acceleration_max = statistics->maxima[3];
    output->angular_velocity_mean = statistics_mean(statistics, 4U);
    output->angular_velocity_min = statistics->minima[4];
    output->angular_velocity_max = statistics->maxima[4];
    output->valid = output->imu_samples != 0U ||
                    output->light_samples != 0U ||
                    output->environment_samples != 0U;
}

/*
 * Channels are accumulated independently: light only when a new in-range LDR
 * measurement was published, temperature/humidity only when a new Si7021
 * measurement was published, and IMU magnitudes on every calibrated valid IMU
 * sample. Window rotation runs on both sample and fault ticks so an IMU fault
 * only removes the IMU channels instead of stalling the 30 s summary.
 */
static void statistics_step_window(struct statistics_state *statistics,
                                   uint32_t timestamp_ms,
                                   struct telemetry_snapshot *snapshot)
{
    if (statistics->initialized &&
        timestamp_ms - statistics->window_start_ms >= STATISTICS_WINDOW_MS) {
        statistics_publish(statistics, &snapshot->statistics);
        statistics_reset(statistics, timestamp_ms);
    }
    if (!statistics->initialized) {
        statistics_reset(statistics, timestamp_ms);
    }
    if (snapshot->light_sample_seq != statistics->last_light_seq) {
        statistics->last_light_seq = snapshot->light_sample_seq;
        statistics_update_channel(statistics, 0U, snapshot->light_percent);
    }
    if (snapshot->environment_sample_seq != statistics->last_environment_seq) {
        statistics->last_environment_seq = snapshot->environment_sample_seq;
        statistics_update_channel(statistics, 1U, snapshot->temperature_c);
        statistics_update_channel(statistics, 2U, snapshot->humidity_percent);
    }
}

static void statistics_add_imu(struct statistics_state *statistics,
                               float acceleration_magnitude,
                               float angular_velocity_magnitude)
{
    statistics_update_channel(statistics, 3U, acceleration_magnitude);
    statistics_update_channel(statistics, 4U, angular_velocity_magnitude);
}

/*
 * Device faults that justify PROBLEM. LDR saturation or an out-of-calibration
 * reading is an environmental condition (FLAGS bit 2 only) and is excluded;
 * only a persistent LDR ADC failure or open/shorted rail (0 / 4095) counts.
 * GPS no-fix never counts (NM5).
 */
static bool health_problem_active(const struct telemetry_snapshot *snapshot)
{
    return (snapshot->flags & (TELEMETRY_FLAG_IMU_INVALID |
                               TELEMETRY_FLAG_SI7021_INVALID)) != 0U ||
           snapshot->ldr_hardware_fault;
}

/*
 * Rules, in precedence order:
 *  1. abnormal (device fault or severe tilt/inversion)  -> PROBLEM
 *  2. dynamic acceleration, rotation, or moderate tilt -> FLYING
 *  3. |a| close to gravity and low rotation            -> IDLE
 * Between the IDLE and FLYING thresholds the current state is held
 * (hysteresis). Each transition must persist for its configured time.
 */
static enum tracker_state detector_update(struct state_detector *detector,
                                          const struct state_detector_config *config,
                                          uint32_t timestamp_ms,
                                          float dynamic_acceleration,
                                          float angular_velocity_magnitude,
                                          bool moderate_tilt,
                                          bool abnormal)
{
    enum tracker_state requested_state;
    uint32_t persistence_ms;

    if (abnormal) {
        requested_state = TRACKER_STATE_PROBLEM;
        persistence_ms = config->problem_persistence_ms;
    } else if (dynamic_acceleration >= config->flying_acceleration_threshold_mps2 ||
               angular_velocity_magnitude >= config->flying_gyro_threshold_dps ||
               moderate_tilt) {
        requested_state = TRACKER_STATE_FLYING;
        persistence_ms = config->flying_persistence_ms;
    } else if (dynamic_acceleration <= config->idle_acceleration_tolerance_mps2 &&
               angular_velocity_magnitude <= config->idle_gyro_threshold_dps) {
        requested_state = TRACKER_STATE_IDLE;
        persistence_ms = config->idle_persistence_ms;
    } else if (detector->state == TRACKER_STATE_PROBLEM) {
        requested_state = TRACKER_STATE_IDLE;
        persistence_ms = config->idle_persistence_ms;
    } else {
        requested_state = detector->state;
        persistence_ms = 0U;
    }

    if (requested_state == detector->state) {
        detector->candidate_valid = false;
        return detector->state;
    }
    if (persistence_ms == 0U) {
        detector->state = requested_state;
        detector->candidate_valid = false;
        return detector->state;
    }
    if (!detector->candidate_valid || detector->candidate_state != requested_state) {
        detector->candidate_state = requested_state;
        detector->candidate_since_ms = timestamp_ms;
        detector->candidate_valid = true;
        return detector->state;
    }
    if (timestamp_ms - detector->candidate_since_ms >= persistence_ms) {
        detector->state = requested_state;
        detector->candidate_valid = false;
    }
    return detector->state;
}

static void calibration_restart(struct imu_calibration *calibration,
                                uint32_t timestamp_ms)
{
    calibration->window_start_ms = timestamp_ms;
    calibration->samples = 0U;
    calibration->sum_gx = 0.0f;
    calibration->sum_gy = 0.0f;
    calibration->sum_gz = 0.0f;
    calibration->sum_accel_magnitude = 0.0f;
    calibration->min_accel_magnitude = 0.0f;
    calibration->max_accel_magnitude = 0.0f;
}

static void calibration_finish(struct imu_calibration *calibration,
                               bool use_measurement)
{
    calibration->gyro_bias_x_dps = 0.0f;
    calibration->gyro_bias_y_dps = 0.0f;
    calibration->gyro_bias_z_dps = 0.0f;
    calibration->gravity_mps2 = STANDARD_GRAVITY_MPS2;

    if (use_measurement && calibration->samples > 0U) {
        float count = (float)calibration->samples;
        float gravity = calibration->sum_accel_magnitude / count;

        calibration->gyro_bias_x_dps = calibration->sum_gx / count;
        calibration->gyro_bias_y_dps = calibration->sum_gy / count;
        calibration->gyro_bias_z_dps = calibration->sum_gz / count;
        if (gravity >= CALIBRATION_MIN_GRAVITY_MPS2 &&
            gravity <= CALIBRATION_MAX_GRAVITY_MPS2) {
            calibration->gravity_mps2 = gravity;
        }
    }
    calibration->active = false;
    calibration->done = true;
}

/*
 * Feed one raw IMU sample to the start-up calibration. Returns true while
 * the calibration is still running. Any movement or non-level tilt restarts
 * the window; if the board is never still and level, the calibration gives up
 * and uses zero bias with standard gravity.
 */
static bool calibration_update(struct imu_calibration *calibration,
                               const struct imu_sample *sample,
                               float accel_magnitude,
                               float raw_gyro_magnitude,
                               float min_cos_level)
{
    bool level_and_1g;

    if (calibration->done) {
        return false;
    }
    if (!calibration->active) {
        calibration->active = true;
        calibration->first_start_ms = sample->timestamp_ms;
        calibration_restart(calibration, sample->timestamp_ms);
    }

    if (calibration->samples == 0U) {
        calibration->min_accel_magnitude = accel_magnitude;
        calibration->max_accel_magnitude = accel_magnitude;
    } else {
        if (accel_magnitude < calibration->min_accel_magnitude) {
            calibration->min_accel_magnitude = accel_magnitude;
        }
        if (accel_magnitude > calibration->max_accel_magnitude) {
            calibration->max_accel_magnitude = accel_magnitude;
        }
    }

    level_and_1g = accel_magnitude >= CALIBRATION_MIN_GRAVITY_MPS2 &&
                   accel_magnitude <= CALIBRATION_MAX_GRAVITY_MPS2 &&
                   sample->az_mps2 > accel_magnitude * min_cos_level;

    if (!level_and_1g ||
        raw_gyro_magnitude > CALIBRATION_MAX_GYRO_DPS ||
        calibration->max_accel_magnitude - calibration->min_accel_magnitude >
            CALIBRATION_MAX_ACCEL_SPREAD_MPS2) {
        if (sample->timestamp_ms - calibration->first_start_ms >=
            calibration->duration_ms * CALIBRATION_TIMEOUT_WINDOWS) {
            calibration_finish(calibration, false);
            return false;
        }
        calibration_restart(calibration, sample->timestamp_ms);
        return true;
    }

    calibration->sum_gx += sample->gx_dps;
    calibration->sum_gy += sample->gy_dps;
    calibration->sum_gz += sample->gz_dps;
    calibration->sum_accel_magnitude += accel_magnitude;
    calibration->samples++;

    if (sample->timestamp_ms - calibration->window_start_ms >=
        calibration->duration_ms) {
        calibration_finish(calibration, true);
        return false;
    }
    return true;
}

void processing_engine_init(struct processing_engine *engine,
                            const struct state_detector_config *config)
{
    memset(engine, 0, sizeof(*engine));
    engine->detector.state = TRACKER_STATE_IDLE;
    engine->detector_config = *config;
    engine->cos_flying_tilt = cosf(config->flying_tilt_deg * DEG_TO_RAD);
    engine->cos_problem_tilt = cosf(config->problem_tilt_deg * DEG_TO_RAD);
    engine->calibration.duration_ms = config->imu_calibration_ms;
    engine->calibration.gravity_mps2 = STANDARD_GRAVITY_MPS2;
    engine->calibration.done = config->imu_calibration_ms == 0U;
    /* Use a virtual local origin until the first valid GPS fix arrives. */
    engine->position.integration_origin_valid = true;
}

void processing_engine_process_imu(struct processing_engine *engine,
                                   const struct imu_sample *sample,
                                   struct telemetry_snapshot *snapshot)
{
    struct imu_sample corrected = *sample;
    float acceleration_magnitude;
    float raw_gyro_magnitude;
    float angular_velocity_magnitude;
    float gravity_mps2;
    bool calibrating;
    bool still_calibrating;
    bool gps_updated = false;
    bool moderate_tilt = false;
    bool severe_tilt = false;

    acceleration_magnitude = magnitude3(sample->ax_mps2, sample->ay_mps2,
                                         sample->az_mps2);
    raw_gyro_magnitude = magnitude3(sample->gx_dps, sample->gy_dps,
                                    sample->gz_dps);
    calibrating = calibration_update(&engine->calibration, sample,
                                     acceleration_magnitude, raw_gyro_magnitude,
                                     engine->cos_flying_tilt);

    corrected.gx_dps -= engine->calibration.gyro_bias_x_dps;
    corrected.gy_dps -= engine->calibration.gyro_bias_y_dps;
    corrected.gz_dps -= engine->calibration.gyro_bias_z_dps;
    gravity_mps2 = engine->calibration.gravity_mps2;
    angular_velocity_magnitude = magnitude3(corrected.gx_dps, corrected.gy_dps,
                                            corrected.gz_dps);

    snapshot->ax_mps2 = corrected.ax_mps2;
    snapshot->ay_mps2 = corrected.ay_mps2;
    snapshot->az_mps2 = corrected.az_mps2;
    snapshot->gx_dps = corrected.gx_dps;
    snapshot->gy_dps = corrected.gy_dps;
    snapshot->gz_dps = corrected.gz_dps;

    if (!snapshot->gps_fix) {
        engine->position.last_gps_valid = false;
    } else if (snapshot->gps_timestamp_ms != engine->position.last_gps_timestamp_ms) {
        position_from_gps(&engine->position, snapshot);
        gps_updated = true;
    }

    if (calibrating) {
        /* Zero initial orientation is defined at the end of calibration. */
        snapshot->flags |= TELEMETRY_FLAG_CALIBRATION_PENDING;
        engine->attitude.initialized = false;
        engine->position.last_imu_valid = false;
    } else {
        snapshot->flags &= ~TELEMETRY_FLAG_CALIBRATION_PENDING;
        attitude_update(&engine->attitude, &corrected);
        engine->attitude.roll_deg = wrap_angle_deg(engine->attitude.roll_deg);
        engine->attitude.pitch_deg = wrap_angle_deg(engine->attitude.pitch_deg);
        engine->attitude.yaw_deg = wrap_angle_deg(engine->attitude.yaw_deg);
        if (!gps_updated) {
            position_integrate_inertial(&engine->position, &engine->attitude,
                                        &corrected, gravity_mps2);
        }
    }

    statistics_step_window(&engine->statistics, sample->timestamp_ms, snapshot);
    if (!calibrating) {
        statistics_add_imu(&engine->statistics, acceleration_magnitude,
                           angular_velocity_magnitude);
    }

    snapshot->roll_deg = engine->attitude.roll_deg;
    snapshot->pitch_deg = engine->attitude.pitch_deg;
    snapshot->yaw_deg = engine->attitude.yaw_deg;
    /* Yaw is currently gyro-only and has no absolute heading reference. */
    snapshot->flags |= TELEMETRY_FLAG_YAW_UNRELIABLE;
    snapshot->local_origin_valid = engine->position.origin_valid;
    snapshot->x_m = engine->position.x_m;
    snapshot->y_m = engine->position.y_m;
    snapshot->z_m = engine->position.z_m;
    snapshot->vx_mps = engine->position.vx_mps;
    snapshot->vy_mps = engine->position.vy_mps;
    snapshot->vz_mps = engine->position.vz_mps;
    if (!snapshot->gps_fix) {
        snapshot->flags |= TELEMETRY_FLAG_INERTIAL_ONLY;
    } else {
        snapshot->flags &= ~TELEMETRY_FLAG_INERTIAL_ONLY;
    }

    /*
     * Tilt from the accelerometer, assuming the sensor Z axis points up when
     * the tracker is level. A multirotor tilts to produce horizontal thrust,
     * so a sustained moderate tilt means flight; a severe tilt or inversion
     * means the drone has tipped over or crashed. The estimate is restricted
     * to quasi-static conditions (|a| close to gravity) so horizontal shaking
     * while flat cannot spoof a severe tilt angle.
     */
    if (acceleration_magnitude > 0.0f &&
        abs_f(acceleration_magnitude - gravity_mps2) <= TILT_MAX_ACCEL_DEV_MPS2) {
        float cos_tilt = corrected.az_mps2 / acceleration_magnitude;

        severe_tilt = cos_tilt <= engine->cos_problem_tilt;
        moderate_tilt = !severe_tilt && cos_tilt <= engine->cos_flying_tilt;
    }

    still_calibrating = calibrating && engine->calibration.samples > 0U;
    snapshot->state = detector_update(
        &engine->detector, &engine->detector_config, sample->timestamp_ms,
        still_calibrating ? 0.0f : abs_f(acceleration_magnitude - gravity_mps2),
        still_calibrating ? 0.0f : angular_velocity_magnitude,
        still_calibrating ? false : moderate_tilt,
        health_problem_active(snapshot) || severe_tilt);

    /* FLAGS bit 4: crash-like event = confirmed severe tilt / inversion. */
    if (severe_tilt && snapshot->state == TRACKER_STATE_PROBLEM) {
        snapshot->flags |= TELEMETRY_FLAG_IMPACT;
    } else if (!severe_tilt &&
               (snapshot->state != TRACKER_STATE_PROBLEM ||
                health_problem_active(snapshot))) {
        snapshot->flags &= ~TELEMETRY_FLAG_IMPACT;
    }
}

void processing_engine_process_imu_fault(struct processing_engine *engine,
                                         uint32_t timestamp_ms,
                                         struct telemetry_snapshot *snapshot)
{
    if (!engine->calibration.done && engine->calibration.active) {
        calibration_restart(&engine->calibration, timestamp_ms);
    }
    statistics_step_window(&engine->statistics, timestamp_ms, snapshot);
    snapshot->flags &= ~TELEMETRY_FLAG_IMPACT;
    snapshot->flags |= TELEMETRY_FLAG_IMU_INVALID;
    snapshot->flags |= TELEMETRY_FLAG_YAW_UNRELIABLE;
    snapshot->state = detector_update(&engine->detector, &engine->detector_config,
                                      timestamp_ms, 0.0f, 0.0f, false, true);
}
