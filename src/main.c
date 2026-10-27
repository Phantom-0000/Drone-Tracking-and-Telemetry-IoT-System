#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "telemetry_snapshot.h"

#define APP_NODE DT_NODELABEL(telemetry_app)

#if !DT_NODE_HAS_STATUS(APP_NODE, okay)
#error "The telemetry_app Device Tree node must be enabled"
#endif

#define IMU_PERIOD_MS (1000U / CONFIG_APP_IMU_SAMPLE_HZ)
#define GPS_PERIOD_MS CONFIG_APP_GPS_UPDATE_PERIOD_MS
#define ENVIRONMENT_PERIOD_MS CONFIG_APP_ENVIRONMENT_PERIOD_MS
#define SIMU_PERIOD_MS CONFIG_APP_SIMU_PERIOD_MS

BUILD_ASSERT(CONFIG_APP_IMU_SAMPLE_HZ == 25,
             "The mandatory baseline uses a 25 Hz IMU rate");
BUILD_ASSERT(CONFIG_APP_SIMU_PERIOD_MS == 100,
             "The mandatory SIMU period is 100 ms");

enum app_mode {
    APP_MODE_TEST = 0,
    APP_MODE_NORMAL,
    APP_MODE_SIMU,
#if CONFIG_APP_ENABLE_ADVANCED_MODE
    APP_MODE_ADVANCED,
#endif
};

K_MSGQ_DEFINE(imu_event_queue, sizeof(struct imu_event), 8, 4);
K_MUTEX_DEFINE(snapshot_mutex);
K_SEM_DEFINE(mode_event, 0, 1);
K_TIMER_DEFINE(output_timer, NULL, NULL);

static struct telemetry_snapshot snapshot;
static atomic_t active_mode;

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
        /* Sensor access is intentionally a Phase 4 integration point. */
        publish_imu_fault(SENSOR_NOT_READY, k_uptime_get_32());
        k_msleep(IMU_PERIOD_MS);
    }
}

static void gps_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        /* UART receive and NMEA parsing are added during sensor integration. */
        k_msleep(GPS_PERIOD_MS);
    }
}

static void environment_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        /* Si7021 and LDR acquisition are added during sensor integration. */
        k_msleep(ENVIRONMENT_PERIOD_MS);
    }
}

static void processing_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

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
        } else {
            snapshot.flags |= TELEMETRY_FLAG_IMU_INVALID;
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

        /* Formatting and the sole VCOM write path are added in Phase 6/7. */
        ARG_UNUSED(local_snapshot);
        (void)atomic_get(&active_mode);
    }
}

static void mode_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        (void)k_sem_take(&mode_event, K_FOREVER);
        /* B1 GPIO handling and mode transitions are added in Phase 6. */
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
    atomic_set(&active_mode, APP_MODE_TEST);
    snapshot.state = TRACKER_STATE_IDLE;
    k_timer_start(&output_timer, K_MSEC(SIMU_PERIOD_MS), K_MSEC(SIMU_PERIOD_MS));

    return 0;
}
