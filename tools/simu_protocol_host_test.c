#include "simu_protocol.h"

#include <stdio.h>
#include <string.h>

int main(void)
{
    struct telemetry_snapshot snapshot = {
        .timestamp_ms = 123456U,
        .ax_mps2 = -0.12f,
        .ay_mps2 = 0.04f,
        .az_mps2 = 9.81f,
        .gx_dps = 0.5f,
        .gy_dps = -1.2f,
        .gz_dps = 0.1f,
        .roll_deg = 2.4f,
        .pitch_deg = -1.1f,
        .yaw_deg = 45.0f,
        .latitude_deg = 40.389462,
        .longitude_deg = -3.627416,
        .altitude_m = 628.4f,
        .satellites = 8U,
        .gps_fix = true,
        .light_percent = 73.5f,
        .temperature_c = 24.6f,
        .humidity_percent = 45.2f,
        .state = TRACKER_STATE_FLYING,
    };
    char frame[SIMU_FRAME_BUFFER_SIZE];

    if (simu_format_frame(&snapshot, 152U, frame) != 0) {
        return 1;
    }
    if (fwrite(frame, 1U, SIMU_FRAME_LENGTH, stdout) != SIMU_FRAME_LENGTH) {
        return 2;
    }
    return 0;
}
