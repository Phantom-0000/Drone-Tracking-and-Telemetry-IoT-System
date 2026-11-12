#ifndef PROCESSING_H
#define PROCESSING_H

#include <stdbool.h>
#include <stdint.h>

#include "telemetry_snapshot.h"

struct attitude_state {
    uint32_t last_timestamp_ms;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    bool initialized;
};

struct position_state {
    bool origin_valid;
    bool integration_origin_valid;
    double origin_latitude_deg;
    double origin_longitude_deg;
    float origin_altitude_m;
    uint32_t last_gps_timestamp_ms;
    bool last_gps_valid;
    double last_gps_latitude_deg;
    double last_gps_longitude_deg;
    float last_gps_altitude_m;
    float x_m;
    float y_m;
    float z_m;
    float vx_mps;
    float vy_mps;
    float vz_mps;
    uint32_t last_imu_timestamp_ms;
    bool last_imu_valid;
};

struct statistics_state {
    uint32_t window_start_ms;
    uint32_t valid_counts[5];
    float sums[5];
    float minima[5];
    float maxima[5];
    /* Last sensor sample sequence numbers already accumulated. */
    uint32_t last_light_seq;
    uint32_t last_environment_seq;
    bool initialized;
};

/*
 * Start-up IMU calibration. While the board is still, the gyroscope bias and
 * the measured gravity magnitude are averaged. Attitude and inertial position
 * are not integrated until the calibration finishes (FLAGS bit 5 is set).
 */
struct imu_calibration {
    uint32_t duration_ms;
    uint32_t first_start_ms;
    uint32_t window_start_ms;
    uint32_t samples;
    float sum_gx;
    float sum_gy;
    float sum_gz;
    float sum_accel_magnitude;
    float min_accel_magnitude;
    float max_accel_magnitude;
    float gyro_bias_x_dps;
    float gyro_bias_y_dps;
    float gyro_bias_z_dps;
    float gravity_mps2;
    bool active;
    bool done;
};

struct state_detector_config {
    float idle_acceleration_tolerance_mps2;
    float idle_gyro_threshold_dps;
    float flying_acceleration_threshold_mps2;
    float flying_gyro_threshold_dps;
    /* Tilt from the Z-up mounting: FLYING above the first, PROBLEM above the second. */
    float flying_tilt_deg;
    float problem_tilt_deg;
    uint32_t idle_persistence_ms;
    uint32_t flying_persistence_ms;
    uint32_t problem_persistence_ms;
    /* Start-up still-calibration window; 0 disables the calibration. */
    uint32_t imu_calibration_ms;
};

struct state_detector {
    enum tracker_state state;
    enum tracker_state candidate_state;
    uint32_t candidate_since_ms;
    bool candidate_valid;
};

struct processing_engine {
    struct attitude_state attitude;
    struct position_state position;
    struct statistics_state statistics;
    struct state_detector detector;
    struct state_detector_config detector_config;
    struct imu_calibration calibration;
    float cos_flying_tilt;
    float cos_problem_tilt;
};

void processing_engine_init(struct processing_engine *engine,
                            const struct state_detector_config *config);

void processing_engine_process_imu(struct processing_engine *engine,
                                   const struct imu_sample *sample,
                                   struct telemetry_snapshot *snapshot);

void processing_engine_process_imu_fault(struct processing_engine *engine,
                                         uint32_t timestamp_ms,
                                         struct telemetry_snapshot *snapshot);

#endif /* PROCESSING_H */
