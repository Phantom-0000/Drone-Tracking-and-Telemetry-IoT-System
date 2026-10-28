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
    uint32_t complete_samples;
    uint32_t valid_counts[5];
    float sums[5];
    float minima[5];
    float maxima[5];
    bool initialized;
};

struct state_detector_config {
    float idle_acceleration_tolerance_mps2;
    float idle_gyro_threshold_dps;
    float flying_acceleration_threshold_mps2;
    float flying_gyro_threshold_dps;
    uint32_t idle_persistence_ms;
    uint32_t flying_persistence_ms;
    uint32_t problem_persistence_ms;
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
