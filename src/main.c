#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "sensor_contract.h"
#include "telemetry_snapshot.h"
#include "processing.h"
#include "mode_manager.h"

#define APP_NODE DT_NODELABEL(telemetry_app)

#if !DT_NODE_HAS_STATUS(APP_NODE, okay)
#error "The telemetry_app Device Tree node must be enabled"
#endif

#define IMU_PERIOD_MS (1000U / CONFIG_APP_IMU_SAMPLE_HZ)
#define GPS_PERIOD_MS CONFIG_APP_GPS_UPDATE_PERIOD_MS
#define ENVIRONMENT_PERIOD_MS CONFIG_APP_ENVIRONMENT_PERIOD_MS

BUILD_ASSERT(CONFIG_APP_IMU_SAMPLE_HZ == 25,
             "The mandatory baseline uses a 25 Hz IMU rate");
BUILD_ASSERT(CONFIG_APP_SIMU_PERIOD_MS == 100,
             "The mandatory SIMU period is 100 ms");

K_MSGQ_DEFINE(imu_event_queue, sizeof(struct imu_event), 8, 4);
K_MSGQ_DEFINE(mode_button_queue, sizeof(uint32_t), 4, 4);
K_MUTEX_DEFINE(snapshot_mutex);
K_TIMER_DEFINE(output_timer, NULL, NULL);

static struct telemetry_snapshot snapshot;
static atomic_t active_mode;
static atomic_t active_led;
static struct processing_engine processing_engine;
static struct mode_manager mode_manager;

#if CONFIG_APP_USE_MOCK_SENSORS
static const struct ldr_calibration ldr_calibration = {
    .dark_raw = CONFIG_APP_LDR_DARK_RAW,
    .maximum_raw = CONFIG_APP_LDR_MAX_RAW,
};
#endif

#if CONFIG_APP_USE_MOCK_SENSORS
static void publish_imu_sample(const struct imu_sample *sample)
{
    struct imu_event event = {
        .kind = IMU_EVENT_SAMPLE,
        .result = SENSOR_OK,
        .sample = *sample,
    };

    if (k_msgq_put(&imu_event_queue, &event, K_NO_WAIT) != 0) {
        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        snapshot.flags |= TELEMETRY_FLAG_IMU_INVALID;
        k_mutex_unlock(&snapshot_mutex);
    }
}
#endif

static void publish_imu_fault(enum sensor_result result, uint32_t timestamp_ms)
{
    struct imu_event event = {
        .kind = IMU_EVENT_FAULT,
        .result = result,
        .sample = {.timestamp_ms = timestamp_ms},
    };

    if (k_msgq_put(&imu_event_queue, &event, K_NO_WAIT) != 0) {
        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        snapshot.flags |= TELEMETRY_FLAG_IMU_INVALID;
        k_mutex_unlock(&snapshot_mutex);
    }
}

static void imu_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
#if CONFIG_APP_USE_MOCK_SENSORS
        struct imu_sample sample;
#endif
        uint32_t timestamp_ms = k_uptime_get_32();

#if CONFIG_APP_USE_MOCK_SENSORS
        if (sensor_mock_read_imu(timestamp_ms, &sample) == SENSOR_OK) {
            publish_imu_sample(&sample);
        } else {
            publish_imu_fault(SENSOR_INVALID, timestamp_ms);
        }
#else
        publish_imu_fault(SENSOR_NOT_READY, timestamp_ms);
#endif
        k_msleep(IMU_PERIOD_MS);
    }
}

static void gps_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
#if CONFIG_APP_USE_MOCK_SENSORS
        struct gps_sample gps;
        enum sensor_result result =
            sensor_mock_read_gps(k_uptime_get_32(), &gps);

        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        if (result == SENSOR_OK) {
            snapshot.latitude_deg = gps.latitude_deg;
            snapshot.longitude_deg = gps.longitude_deg;
            snapshot.altitude_m = gps.altitude_m;
            snapshot.satellites = gps.satellites;
            snapshot.gps_utc_hour = gps.utc_hour;
            snapshot.gps_utc_minute = gps.utc_minute;
            snapshot.gps_utc_second = gps.utc_second;
            snapshot.gps_fix = gps.fix_valid;
            snapshot.gps_utc_valid = gps.utc_valid;
            snapshot.gps_timestamp_ms = gps.timestamp_ms;
            if (gps.fix_valid) {
                snapshot.flags &= ~TELEMETRY_FLAG_GPS_INVALID;
            } else {
                snapshot.flags |= TELEMETRY_FLAG_GPS_INVALID;
            }
        } else {
            snapshot.flags |= TELEMETRY_FLAG_GPS_INVALID;
        }
        k_mutex_unlock(&snapshot_mutex);
#else
        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        snapshot.flags |= TELEMETRY_FLAG_GPS_INVALID;
        k_mutex_unlock(&snapshot_mutex);
#endif
        k_msleep(GPS_PERIOD_MS);
    }
}

static void environment_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
#if CONFIG_APP_USE_MOCK_SENSORS
        uint32_t timestamp_ms = k_uptime_get_32();
        struct environment_sample environment;
        struct ldr_sample ldr;
        float light_percent;
        enum sensor_result environment_result =
            sensor_mock_read_environment(timestamp_ms, &environment);
        enum sensor_result ldr_result =
            sensor_mock_read_ldr(timestamp_ms, &ldr);

        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        if (environment_result == SENSOR_OK) {
            snapshot.temperature_c = environment.temperature_c;
            snapshot.humidity_percent = environment.humidity_percent;
            snapshot.flags &= ~TELEMETRY_FLAG_SI7021_INVALID;
        } else {
            snapshot.flags |= TELEMETRY_FLAG_SI7021_INVALID;
        }

        if (ldr_result == SENSOR_OK &&
            ldr_map_percent(ldr.adc_raw, timestamp_ms, &ldr_calibration,
                            &ldr, &light_percent) == SENSOR_OK) {
            snapshot.light_percent = light_percent;
            snapshot.flags &= ~TELEMETRY_FLAG_LDR_INVALID;
        } else {
            snapshot.flags |= TELEMETRY_FLAG_LDR_INVALID;
        }
        k_mutex_unlock(&snapshot_mutex);
#else
        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        snapshot.flags |= TELEMETRY_FLAG_SI7021_INVALID |
                          TELEMETRY_FLAG_LDR_INVALID;
        k_mutex_unlock(&snapshot_mutex);
#endif
        k_msleep(ENVIRONMENT_PERIOD_MS);
    }
}

static void processing_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct state_detector_config detector_config = {
        .idle_acceleration_tolerance_mps2 =
            (float)CONFIG_APP_IDLE_ACCEL_TOLERANCE_MILLI_MPS2 / 1000.0f,
        .idle_gyro_threshold_dps = (float)CONFIG_APP_IDLE_GYRO_THRESHOLD_DPS,
        .flying_acceleration_threshold_mps2 =
            (float)CONFIG_APP_FLYING_ACCEL_THRESHOLD_MILLI_MPS2 / 1000.0f,
        .flying_gyro_threshold_dps = (float)CONFIG_APP_FLYING_GYRO_THRESHOLD_DPS,
        .idle_persistence_ms = CONFIG_APP_IDLE_PERSISTENCE_MS,
        .flying_persistence_ms = CONFIG_APP_FLYING_PERSISTENCE_MS,
        .problem_persistence_ms = CONFIG_APP_PROBLEM_PERSISTENCE_MS,
    };

    processing_engine_init(&processing_engine, &detector_config);

    while (true) {
        struct imu_event event;

        (void)k_msgq_get(&imu_event_queue, &event, K_FOREVER);
        k_mutex_lock(&snapshot_mutex, K_FOREVER);

        snapshot.timestamp_ms = event.sample.timestamp_ms;
        if (event.kind == IMU_EVENT_SAMPLE && event.result == SENSOR_OK) {
            snapshot.ax_mps2 = event.sample.ax_mps2;
            snapshot.ay_mps2 = event.sample.ay_mps2;
            snapshot.az_mps2 = event.sample.az_mps2;
            snapshot.gx_dps = event.sample.gx_dps;
            snapshot.gy_dps = event.sample.gy_dps;
            snapshot.gz_dps = event.sample.gz_dps;
            snapshot.flags &= ~TELEMETRY_FLAG_IMU_INVALID;
            processing_engine_process_imu(&processing_engine,
                                          &event.sample, &snapshot);
        } else {
            snapshot.flags |= TELEMETRY_FLAG_IMU_INVALID;
            processing_engine_process_imu_fault(&processing_engine,
                                                event.sample.timestamp_ms,
                                                &snapshot);
        }

        k_mutex_unlock(&snapshot_mutex);
    }
}

static void output_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        struct telemetry_snapshot local_snapshot;

        (void)k_timer_status_sync(&output_timer);
        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        local_snapshot = snapshot;
        k_mutex_unlock(&snapshot_mutex);

        /*
         * Phase 6 owns the mode-dependent schedule and LED policy. Phase 7
         * adds the actual TEST/NORMAL/SIMU formatters to this sole VCOM
         * writer.
         */
        enum app_mode mode = (enum app_mode)atomic_get(&active_mode);
        enum tracker_state state = local_snapshot.state;
        atomic_set(&active_led,
                   mode_manager_led_for(mode, state));
        ARG_UNUSED(local_snapshot);
        ARG_UNUSED(mode);
    }
}

/*
 * Hardware GPIO setup is intentionally deferred until the B1 pin is frozen.
 * The eventual GPIO callback can call this function from thread or ISR
 * context; mode_thread performs the debounced transition and timer update.
 */
void app_b1_button_pressed(uint32_t timestamp_ms)
{
    (void)k_msgq_put(&mode_button_queue, &timestamp_ms, K_NO_WAIT);
}

static void mode_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        uint32_t timestamp_ms;

        (void)k_msgq_get(&mode_button_queue, &timestamp_ms, K_FOREVER);
        if (!mode_manager_accept_button(&mode_manager, timestamp_ms)) {
            continue;
        }

        mode_manager.mode = mode_manager_next_mode(mode_manager.mode);
        atomic_set(&active_mode, mode_manager.mode);
        enum tracker_state state;

        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        state = snapshot.state;
        k_mutex_unlock(&snapshot_mutex);
        atomic_set(&active_led, mode_manager_led_for(mode_manager.mode, state));
        k_timer_start(&output_timer,
                      K_MSEC(mode_manager_period_ms(mode_manager.mode)),
                      K_MSEC(mode_manager_period_ms(mode_manager.mode)));
    }
}

K_THREAD_DEFINE(imu_thread_id, 1536, imu_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(gps_thread_id, 1536, gps_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(environment_thread_id, 1536, environment_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(processing_thread_id, 2048, processing_thread, NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(output_thread_id, 2048, output_thread, NULL, NULL, NULL, 8, 0, 0);
K_THREAD_DEFINE(mode_thread_id, 1024, mode_thread, NULL, NULL, NULL, 9, 0, 0);

int main(void)
{
    mode_manager_init(&mode_manager, CONFIG_APP_MODE_BUTTON_DEBOUNCE_MS);
    atomic_set(&active_mode, mode_manager.mode);
    atomic_set(&active_led,
               mode_manager_led_for(mode_manager.mode, TRACKER_STATE_IDLE));
    snapshot.state = TRACKER_STATE_IDLE;
    k_timer_start(&output_timer,
                  K_MSEC(mode_manager_period_ms(mode_manager.mode)),
                  K_MSEC(mode_manager_period_ms(mode_manager.mode)));

    return 0;
}
