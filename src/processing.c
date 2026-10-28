#include "processing.h"

#include <math.h>
#include <string.h>

#define EARTH_METERS_PER_DEGREE_LATITUDE 110540.0
#define EARTH_METERS_PER_DEGREE_LONGITUDE 111320.0
#define STATISTICS_WINDOW_MS 30000U

static float magnitude3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
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

    if (position->last_gps_valid &&
        snapshot->gps_timestamp_ms != position->last_gps_timestamp_ms) {
        float dt = clamp_dt_seconds(position->last_gps_timestamp_ms,
                                    snapshot->gps_timestamp_ms);
        if (dt > 0.0f) {
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
                                        const struct imu_sample *sample)
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
    linear_z = world_z - 9.80665f;
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
    memset(statistics, 0, sizeof(*statistics));
    statistics->window_start_ms = timestamp_ms;
    statistics->initialized = true;
}

static void statistics_add(struct statistics_state *statistics,
                           uint32_t timestamp_ms,
                           const struct telemetry_snapshot *snapshot,
                           float acceleration_magnitude,
                           float angular_velocity_magnitude)
{
    float values[5] = {
        snapshot->light_percent,
        snapshot->temperature_c,
        snapshot->humidity_percent,
        acceleration_magnitude,
        angular_velocity_magnitude,
    };
    bool environment_valid =
        (snapshot->flags & (TELEMETRY_FLAG_LDR_INVALID |
                            TELEMETRY_FLAG_SI7021_INVALID)) == 0U;
    bool valid[5] = {
        environment_valid,
        environment_valid,
        environment_valid,
        true,
        true,
    };
    size_t index;

    if (!statistics->initialized) {
        statistics_reset(statistics, timestamp_ms);
    }
    for (index = 0U; index < 5U; ++index) {
        uint32_t count;

        if (!valid[index]) {
            continue;
        }
        count = statistics->valid_counts[index];
        statistics->sums[index] += values[index];
        if (count == 0U || values[index] < statistics->minima[index]) {
            statistics->minima[index] = values[index];
        }
        if (count == 0U || values[index] > statistics->maxima[index]) {
            statistics->maxima[index] = values[index];
        }
        statistics->valid_counts[index] = count + 1U;
    }
    if (environment_valid) {
        statistics->complete_samples++;
    }
}

static void statistics_publish(const struct statistics_state *statistics,
                               struct telemetry_statistics *output)
{
    if (statistics->complete_samples == 0U) {
        output->valid = false;
        return;
    }

    output->window_end_ms = statistics->window_start_ms + STATISTICS_WINDOW_MS;
    output->valid_samples = statistics->complete_samples;
    output->light_mean = statistics->sums[0] / statistics->valid_counts[0];
    output->light_min = statistics->minima[0];
    output->light_max = statistics->maxima[0];
    output->temperature_mean = statistics->sums[1] / statistics->valid_counts[1];
    output->temperature_min = statistics->minima[1];
    output->temperature_max = statistics->maxima[1];
    output->humidity_mean = statistics->sums[2] / statistics->valid_counts[2];
    output->humidity_min = statistics->minima[2];
    output->humidity_max = statistics->maxima[2];
    output->acceleration_mean = statistics->sums[3] / statistics->valid_counts[3];
    output->acceleration_min = statistics->minima[3];
    output->acceleration_max = statistics->maxima[3];
    output->angular_velocity_mean = statistics->sums[4] / statistics->valid_counts[4];
    output->angular_velocity_min = statistics->minima[4];
    output->angular_velocity_max = statistics->maxima[4];
    output->valid = true;
}

static bool health_problem_active(uint16_t flags)
{
    return (flags & (TELEMETRY_FLAG_IMU_INVALID |
                     TELEMETRY_FLAG_LDR_INVALID |
                     TELEMETRY_FLAG_SI7021_INVALID |
                     TELEMETRY_FLAG_IMPACT)) != 0U;
}

static enum tracker_state detector_update(struct state_detector *detector,
                                          const struct state_detector_config *config,
                                          uint32_t timestamp_ms,
                                          float acceleration_magnitude,
                                          float angular_velocity_magnitude,
                                          uint16_t flags)
{
    enum tracker_state requested_state;
    uint32_t persistence_ms;

    if (health_problem_active(flags)) {
        requested_state = TRACKER_STATE_PROBLEM;
        persistence_ms = config->problem_persistence_ms;
    } else if (fabsf(acceleration_magnitude - 9.80665f) <=
                   config->idle_acceleration_tolerance_mps2 &&
               angular_velocity_magnitude <= config->idle_gyro_threshold_dps) {
        requested_state = TRACKER_STATE_IDLE;
        persistence_ms = config->idle_persistence_ms;
    } else if (fabsf(acceleration_magnitude - 9.80665f) >=
                   config->flying_acceleration_threshold_mps2 ||
               angular_velocity_magnitude >= config->flying_gyro_threshold_dps) {
        requested_state = TRACKER_STATE_FLYING;
        persistence_ms = config->flying_persistence_ms;
    } else {
        requested_state = detector->state;
        persistence_ms = 0U;
    }

    if (requested_state == detector->state || persistence_ms == 0U) {
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

void processing_engine_init(struct processing_engine *engine,
                            const struct state_detector_config *config)
{
    memset(engine, 0, sizeof(*engine));
    engine->detector.state = TRACKER_STATE_IDLE;
    engine->detector_config = *config;
    /* Use a virtual local origin until the first valid GPS fix arrives. */
    engine->position.integration_origin_valid = true;
}

void processing_engine_process_imu(struct processing_engine *engine,
                                   const struct imu_sample *sample,
                                   struct telemetry_snapshot *snapshot)
{
    float acceleration_magnitude;
    float angular_velocity_magnitude;
    bool gps_updated = false;

    attitude_update(&engine->attitude, sample);
    engine->attitude.roll_deg = wrap_angle_deg(engine->attitude.roll_deg);
    engine->attitude.pitch_deg = wrap_angle_deg(engine->attitude.pitch_deg);
    engine->attitude.yaw_deg = wrap_angle_deg(engine->attitude.yaw_deg);
    if (snapshot->gps_fix &&
        snapshot->gps_timestamp_ms != engine->position.last_gps_timestamp_ms) {
        position_from_gps(&engine->position, snapshot);
        gps_updated = true;
    }
    if (!gps_updated) {
        position_integrate_inertial(&engine->position, &engine->attitude, sample);
    }

    acceleration_magnitude = magnitude3(sample->ax_mps2, sample->ay_mps2,
                                         sample->az_mps2);
    angular_velocity_magnitude = magnitude3(sample->gx_dps, sample->gy_dps,
                                             sample->gz_dps);
    if (engine->statistics.initialized &&
        sample->timestamp_ms - engine->statistics.window_start_ms >=
            STATISTICS_WINDOW_MS) {
        statistics_publish(&engine->statistics, &snapshot->statistics);
        statistics_reset(&engine->statistics, sample->timestamp_ms);
    }
    statistics_add(&engine->statistics, sample->timestamp_ms, snapshot,
                   acceleration_magnitude, angular_velocity_magnitude);

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
    snapshot->state = detector_update(&engine->detector, &engine->detector_config,
                                      sample->timestamp_ms,
                                      acceleration_magnitude,
                                      angular_velocity_magnitude,
                                      snapshot->flags);
}

void processing_engine_process_imu_fault(struct processing_engine *engine,
                                         uint32_t timestamp_ms,
                                         struct telemetry_snapshot *snapshot)
{
    snapshot->flags |= TELEMETRY_FLAG_IMU_INVALID;
    snapshot->flags |= TELEMETRY_FLAG_YAW_UNRELIABLE;
    snapshot->state = detector_update(&engine->detector, &engine->detector_config,
                                      timestamp_ms, 0.0f, 0.0f,
                                      snapshot->flags);
}
