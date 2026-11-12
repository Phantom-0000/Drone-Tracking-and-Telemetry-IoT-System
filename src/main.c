
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "sensor_contract.h"
#include "telemetry_snapshot.h"
#include "processing.h"
#include "mode_manager.h"
#include "simu_protocol.h"
#include "output_formatter.h"

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
/* The IDLE and FLYING bands must not overlap (hysteresis between them). */
BUILD_ASSERT(CONFIG_APP_FLYING_ACCEL_THRESHOLD_MILLI_MPS2 >
                 CONFIG_APP_IDLE_ACCEL_TOLERANCE_MILLI_MPS2,
             "FLYING acceleration threshold must exceed the IDLE tolerance");
BUILD_ASSERT(CONFIG_APP_FLYING_GYRO_THRESHOLD_DPS >
                 CONFIG_APP_IDLE_GYRO_THRESHOLD_DPS,
             "FLYING gyro threshold must exceed the IDLE threshold");
BUILD_ASSERT(CONFIG_APP_PROBLEM_TILT_DEG > CONFIG_APP_FLYING_TILT_DEG,
             "PROBLEM tilt must exceed the FLYING tilt");

/* Consecutive read failures before a sensor is treated as faulty. */
#define SENSOR_FAULT_STREAK 3U

K_MSGQ_DEFINE(imu_event_queue, sizeof(struct imu_event), 8, 4);
K_MSGQ_DEFINE(mode_button_queue, sizeof(uint32_t), 4, 4);
K_MUTEX_DEFINE(snapshot_mutex);
K_MUTEX_DEFINE(led_mutex);
K_TIMER_DEFINE(output_timer, NULL, NULL);

static struct telemetry_snapshot snapshot;
static atomic_t active_mode;
static atomic_t active_led;
static struct processing_engine processing_engine;
static struct mode_manager mode_manager;
static uint16_t simu_sequence;

#if DT_HAS_CHOSEN(zephyr_console)
static const struct device *const console_uart_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
#define VCOM_IRQ_TX 1
/*
 * Interrupt-driven VCOM transmission: output_thread hands a buffer to the
 * UART ISR and blocks on a semaphore until the last byte has left the shift
 * register. The CPU is free (idle/WFI) while the bytes are on the wire
 * instead of spinning in uart_poll_out().
 */
static K_SEM_DEFINE(vcom_tx_done, 0, 1);
static const uint8_t *vcom_tx_data;
static size_t vcom_tx_length;
static size_t vcom_tx_offset;
static bool vcom_irq_ready;

static void vcom_uart_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) > 0 && uart_irq_is_pending(dev) > 0) {
        if (uart_irq_tx_ready(dev) <= 0) {
            break;
        }
        if (vcom_tx_offset < vcom_tx_length) {
            int sent = uart_fifo_fill(dev, &vcom_tx_data[vcom_tx_offset],
                                      (int)(vcom_tx_length - vcom_tx_offset));

            if (sent <= 0) {
                break;
            }
            vcom_tx_offset += (size_t)sent;
        } else {
            /* Transmission complete (TC) after the last byte. */
            uart_irq_tx_disable(dev);
            k_sem_give(&vcom_tx_done);
            break;
        }
    }
}
#endif
#endif

static void emit_vcom_bytes(const char *data, size_t length)
{
    if (length == 0U) {
        return;
    }
#if DT_HAS_CHOSEN(zephyr_console)
#if defined(VCOM_IRQ_TX)
    if (vcom_irq_ready) {
        /* 115200 8N1 sends ~11.5 bytes/ms; allow a generous margin. */
        k_timeout_t timeout = K_MSEC((length / 10U) + 50U);

        k_sem_reset(&vcom_tx_done);
        vcom_tx_data = (const uint8_t *)data;
        vcom_tx_length = length;
        vcom_tx_offset = 0U;
        uart_irq_tx_enable(console_uart_dev);
        if (k_sem_take(&vcom_tx_done, timeout) != 0) {
            uart_irq_tx_disable(console_uart_dev);
        }
        return;
    }
#endif
    if (device_is_ready(console_uart_dev)) {
        for (size_t i = 0U; i < length; ++i) {
            uart_poll_out(console_uart_dev, (unsigned char)data[i]);
        }
        return;
    }
#endif
    printk("%.*s", (int)length, data);
}

static void init_vcom_tx(void)
{
#if defined(VCOM_IRQ_TX)
    if (device_is_ready(console_uart_dev) &&
        uart_irq_callback_user_data_set(console_uart_dev, vcom_uart_isr,
                                        NULL) == 0) {
        uart_irq_tx_disable(console_uart_dev);
        vcom_irq_ready = true;
    }
#endif
}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(red_led_3), okay)
#define LED_RED_NODE DT_NODELABEL(red_led_3)
#else
#define LED_RED_NODE DT_NODELABEL(green_led_3)
#endif

static const struct gpio_dt_spec b1_button =
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec led_red =
    GPIO_DT_SPEC_GET(LED_RED_NODE, gpios);
static const struct gpio_dt_spec led_green =
    GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_2), gpios);
static const struct gpio_dt_spec led_blue =
    GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led_1), gpios);
static struct gpio_callback b1_button_cb_data;
static atomic_t leds_ready;
static enum app_led_color applied_led_color;
static bool led_color_applied;

/*
 * Called from the processing, output and mode threads. The mutex keeps the
 * three GPIO writes of one colour together, and the GPIOs are only touched
 * when the colour actually changes.
 */
static void apply_led_color(enum app_led_color color)
{
    int r = 0;
    int g = 0;
    int b = 0;

    atomic_set(&active_led, color);
    if (atomic_get(&leds_ready) == 0) {
        return;
    }

    k_mutex_lock(&led_mutex, K_FOREVER);
    if (led_color_applied && applied_led_color == color) {
        k_mutex_unlock(&led_mutex);
        return;
    }

    switch (color) {
    case APP_LED_BLUE:
        b = 1;
        break;
    case APP_LED_GREEN:
        g = 1;
        break;
    case APP_LED_YELLOW:
        r = 1;
        g = 1;
        break;
    case APP_LED_RED:
        r = 1;
        break;
    case APP_LED_MAGENTA:
        r = 1;
        b = 1;
        break;
    case APP_LED_WHITE:
        r = 1;
        g = 1;
        b = 1;
        break;
    case APP_LED_OFF:
    default:
        break;
    }

    (void)gpio_pin_set_dt(&led_red, r);
    (void)gpio_pin_set_dt(&led_green, g);
    (void)gpio_pin_set_dt(&led_blue, b);
    applied_led_color = color;
    led_color_applied = true;
    k_mutex_unlock(&led_mutex);
}

static const struct ldr_calibration ldr_calibration = {
    .dark_raw = CONFIG_APP_LDR_DARK_RAW,
    .maximum_raw = CONFIG_APP_LDR_MAX_RAW,
};

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
        struct imu_sample sample;
        uint32_t timestamp_ms = k_uptime_get_32();
#if CONFIG_APP_USE_MOCK_SENSORS
        enum sensor_result result = sensor_mock_read_imu(timestamp_ms, &sample);
#else
        enum sensor_result result = sensor_hw_read_imu(timestamp_ms, &sample);
#endif

        if (result == SENSOR_OK) {
            publish_imu_sample(&sample);
        } else {
            publish_imu_fault(result, timestamp_ms);
        }
        k_msleep(IMU_PERIOD_MS);
    }
}

static void gps_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        struct gps_sample gps;
        uint32_t timestamp_ms = k_uptime_get_32();
#if CONFIG_APP_USE_MOCK_SENSORS
        enum sensor_result result = sensor_mock_read_gps(timestamp_ms, &gps);
#else
        enum sensor_result result = sensor_hw_read_gps(timestamp_ms, &gps);
#endif

        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        if (result == SENSOR_OK) {
            snapshot.gps_utc_hour = gps.utc_hour;
            snapshot.gps_utc_minute = gps.utc_minute;
            snapshot.gps_utc_second = gps.utc_second;
            snapshot.gps_fix = gps.fix_valid;
            snapshot.gps_utc_valid = gps.utc_valid;
            snapshot.gps_timestamp_ms = gps.timestamp_ms;
            if (gps.fix_valid) {
                snapshot.latitude_deg = gps.latitude_deg;
                snapshot.longitude_deg = gps.longitude_deg;
                snapshot.altitude_m = gps.altitude_m;
                snapshot.satellites = gps.satellites;
                snapshot.flags &= ~TELEMETRY_FLAG_GPS_INVALID;
            } else {
                snapshot.satellites = 0U;
                snapshot.flags |= TELEMETRY_FLAG_GPS_INVALID;
            }
        } else {
            snapshot.gps_fix = false;
            snapshot.gps_utc_valid = false;
            snapshot.satellites = 0U;
            snapshot.flags |= TELEMETRY_FLAG_GPS_INVALID;
        }
        k_mutex_unlock(&snapshot_mutex);
        k_msleep(GPS_PERIOD_MS);
    }
}

static void environment_thread(void *p1, void *p2, void *p3)
{
    uint32_t env_fail_streak = 0U;
    uint32_t ldr_fail_streak = 0U;

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    k_msleep(60);

    while (true) {
        uint32_t timestamp_ms = k_uptime_get_32();
        struct environment_sample environment;
        struct ldr_sample ldr;
        float light_percent = 0.0f;
        enum sensor_result ldr_map_result = SENSOR_INVALID;
#if CONFIG_APP_USE_MOCK_SENSORS
        enum sensor_result environment_result =
            sensor_mock_read_environment(timestamp_ms, &environment);
        enum sensor_result ldr_result =
            sensor_mock_read_ldr(timestamp_ms, &ldr);
#else
        enum sensor_result environment_result =
            sensor_hw_read_environment(timestamp_ms, &environment);
        enum sensor_result ldr_result =
            sensor_hw_read_ldr(timestamp_ms, &ldr);
#endif

        if (ldr_result == SENSOR_OK) {
            ldr_map_result = ldr_map_percent(ldr.adc_raw, timestamp_ms,
                                             &ldr_calibration, &ldr,
                                             &light_percent);
        }

        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        if (environment_result == SENSOR_OK) {
            env_fail_streak = 0U;
            snapshot.temperature_c = environment.temperature_c;
            snapshot.humidity_percent = environment.humidity_percent;
            snapshot.environment_sample_seq++;
            snapshot.flags &= ~TELEMETRY_FLAG_SI7021_INVALID;
        } else {
            env_fail_streak++;
            if (env_fail_streak >= SENSOR_FAULT_STREAK) {
                snapshot.flags |= TELEMETRY_FLAG_SI7021_INVALID;
            }
        }

        if (ldr_result == SENSOR_OK && ldr_map_result == SENSOR_OK) {
            ldr_fail_streak = 0U;
            snapshot.ldr_hardware_fault = false;
            snapshot.light_percent = light_percent;
            snapshot.light_sample_seq++;
            snapshot.flags &= ~TELEMETRY_FLAG_LDR_INVALID;
        } else if (ldr_result == SENSOR_OK &&
                   ldr_map_result == SENSOR_OVERFLOW) {
            /* Out of calibrated range (not railed): report clamped 0/100 % and
             * set FLAGS bit 2, without triggering PROBLEM or polluting stats. */
            ldr_fail_streak = 0U;
            snapshot.ldr_hardware_fault = false;
            snapshot.light_percent = light_percent;
            snapshot.flags |= TELEMETRY_FLAG_LDR_INVALID;
        } else {
            /* ADC read failure or railed divider (0 / 4095): keep the last
             * value, set FLAGS bit 2, and declare PROBLEM if it persists. */
            ldr_fail_streak++;
            snapshot.flags |= TELEMETRY_FLAG_LDR_INVALID;
            if (ldr_fail_streak >= SENSOR_FAULT_STREAK) {
                snapshot.ldr_hardware_fault = true;
            }
        }
        k_mutex_unlock(&snapshot_mutex);
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
        .flying_tilt_deg = (float)CONFIG_APP_FLYING_TILT_DEG,
        .problem_tilt_deg = (float)CONFIG_APP_PROBLEM_TILT_DEG,
        .idle_persistence_ms = CONFIG_APP_IDLE_PERSISTENCE_MS,
        .flying_persistence_ms = CONFIG_APP_FLYING_PERSISTENCE_MS,
        .problem_persistence_ms = CONFIG_APP_PROBLEM_PERSISTENCE_MS,
        .imu_calibration_ms = CONFIG_APP_IMU_CALIBRATION_MS,
    };

    processing_engine_init(&processing_engine, &detector_config);

    while (true) {
        struct imu_event event;

        (void)k_msgq_get(&imu_event_queue, &event, K_FOREVER);
        k_mutex_lock(&snapshot_mutex, K_FOREVER);

        snapshot.timestamp_ms = event.sample.timestamp_ms;
        if (event.kind == IMU_EVENT_SAMPLE && event.result == SENSOR_OK) {
            /* processing publishes the bias-corrected IMU values. */
            snapshot.flags &= ~TELEMETRY_FLAG_IMU_INVALID;
            processing_engine_process_imu(&processing_engine,
                                          &event.sample, &snapshot);
        } else {
            snapshot.flags |= TELEMETRY_FLAG_IMU_INVALID;
            processing_engine_process_imu_fault(&processing_engine,
                                                event.sample.timestamp_ms,
                                                &snapshot);
        }

        enum tracker_state state = snapshot.state;

        k_mutex_unlock(&snapshot_mutex);
        apply_led_color(
            mode_manager_led_for((enum app_mode)atomic_get(&active_mode),
                                 state));
    }
}

static void output_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        struct telemetry_snapshot local_snapshot;
        static enum app_mode last_output_mode = APP_MODE_TEST;
        static bool output_mode_initialized;
        static bool stats_baseline_initialized;
        static uint32_t last_stats_window_end_ms;
        static char report[CONFIG_APP_REPORT_BUFFER_SIZE];
        bool include_statistics = false;
        int report_length = 0;

        (void)k_timer_status_sync(&output_timer);
        uint32_t output_timestamp_ms = k_uptime_get_32();
        k_mutex_lock(&snapshot_mutex, K_FOREVER);
        local_snapshot = snapshot;
        k_mutex_unlock(&snapshot_mutex);
        local_snapshot.timestamp_ms = output_timestamp_ms;

        /*
         * output_thread is the only VCOM writer: it applies the mode LED
         * policy and emits the TEST, NORMAL or SIMU output for this period.
         */
        enum app_mode mode = (enum app_mode)atomic_get(&active_mode);
        enum tracker_state state = local_snapshot.state;
        apply_led_color(mode_manager_led_for(mode, state));

        if (!output_mode_initialized || mode != last_output_mode) {
            output_mode_initialized = true;
            last_output_mode = mode;
            stats_baseline_initialized = mode == APP_MODE_NORMAL;
            last_stats_window_end_ms = local_snapshot.statistics.window_end_ms;
        }

        if (mode == APP_MODE_SIMU) {
            char frame[SIMU_FRAME_BUFFER_SIZE];

            if (simu_format_frame(&local_snapshot, simu_sequence, frame) == 0) {
                emit_vcom_bytes(frame, SIMU_FRAME_LENGTH);
                simu_sequence++;
            }
        } else if (mode == APP_MODE_TEST) {
            report_length = output_format_test_report(
                &local_snapshot, report, sizeof(report));
        } else if (mode == APP_MODE_NORMAL) {
            if (local_snapshot.statistics.valid &&
                stats_baseline_initialized &&
                local_snapshot.statistics.window_end_ms !=
                    last_stats_window_end_ms) {
                include_statistics = true;
                last_stats_window_end_ms =
                    local_snapshot.statistics.window_end_ms;
            }
            report_length = output_format_normal_report(
                &local_snapshot, include_statistics,
                report, sizeof(report));
#if CONFIG_APP_ENABLE_ADVANCED_MODE
        } else if (mode == APP_MODE_ADVANCED) {
            report_length = output_format_normal_report(
                &local_snapshot, false, report, sizeof(report));
#endif
        }

        if (report_length > 0) {
            emit_vcom_bytes(report, (size_t)report_length);
        }
    }
}

/*
 * Called from the B1 GPIO interrupt (b1_button_isr). It only queues the
 * press timestamp; mode_thread performs the debounced transition and the
 * output timer update in thread context.
 */
void app_b1_button_pressed(uint32_t timestamp_ms)
{
    (void)k_msgq_put(&mode_button_queue, &timestamp_ms, K_NO_WAIT);
}

static void b1_button_isr(const struct device *dev,
                          struct gpio_callback *cb,
                          uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    app_b1_button_pressed(k_uptime_get_32());
}

static void init_gpio_peripherals(void)
{
    if (led_red.port != NULL && led_green.port != NULL &&
        led_blue.port != NULL &&
        gpio_is_ready_dt(&led_red) &&
        gpio_is_ready_dt(&led_green) &&
        gpio_is_ready_dt(&led_blue) &&
        gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE) == 0 &&
        gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE) == 0 &&
        gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE) == 0) {
        atomic_set(&leds_ready, 1);
    }

    if (b1_button.port != NULL &&
        gpio_is_ready_dt(&b1_button) &&
        gpio_pin_configure_dt(&b1_button, GPIO_INPUT) == 0 &&
        gpio_pin_interrupt_configure_dt(&b1_button,
                                        GPIO_INT_EDGE_TO_ACTIVE) == 0) {
        gpio_init_callback(&b1_button_cb_data, b1_button_isr,
                           BIT(b1_button.pin));
        (void)gpio_add_callback(b1_button.port, &b1_button_cb_data);
    }
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
        apply_led_color(mode_manager_led_for(mode_manager.mode, state));
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
    init_vcom_tx();
    init_gpio_peripherals();
    mode_manager_init(&mode_manager, CONFIG_APP_MODE_BUTTON_DEBOUNCE_MS);
    atomic_set(&active_mode, mode_manager.mode);
    apply_led_color(
        mode_manager_led_for(mode_manager.mode, TRACKER_STATE_IDLE));
    snapshot.state = TRACKER_STATE_IDLE;
    snapshot.flags = TELEMETRY_FLAG_YAW_UNRELIABLE |
                     TELEMETRY_FLAG_GPS_INVALID |
                     TELEMETRY_FLAG_LDR_INVALID |
                     TELEMETRY_FLAG_SI7021_INVALID |
                     TELEMETRY_FLAG_INERTIAL_ONLY;
    if (CONFIG_APP_IMU_CALIBRATION_MS > 0) {
        snapshot.flags |= TELEMETRY_FLAG_CALIBRATION_PENDING;
    }
    k_timer_start(&output_timer,
                  K_MSEC(mode_manager_period_ms(mode_manager.mode)),
                  K_MSEC(mode_manager_period_ms(mode_manager.mode)));

    return 0;
}
