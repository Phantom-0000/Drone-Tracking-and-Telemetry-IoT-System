#include "mode_manager.h"

static bool elapsed_at_least(uint32_t now_ms, uint32_t then_ms,
                             uint32_t interval_ms)
{
    return (uint32_t)(now_ms - then_ms) >= interval_ms;
}

void mode_manager_init(struct mode_manager *manager,
                       uint32_t button_debounce_ms)
{
    manager->mode = APP_MODE_TEST;
    manager->button_debounce_ms = button_debounce_ms;
    manager->last_button_ms = 0U;
    manager->button_timestamp_valid = false;
}

bool mode_manager_accept_button(struct mode_manager *manager,
                                uint32_t timestamp_ms)
{
    if (manager->button_timestamp_valid &&
        !elapsed_at_least(timestamp_ms, manager->last_button_ms,
                          manager->button_debounce_ms)) {
        return false;
    }

    manager->last_button_ms = timestamp_ms;
    manager->button_timestamp_valid = true;
    return true;
}

enum app_mode mode_manager_next_mode(enum app_mode mode)
{
    switch (mode) {
    case APP_MODE_TEST:
        return APP_MODE_NORMAL;
    case APP_MODE_NORMAL:
        return APP_MODE_SIMU;
    case APP_MODE_SIMU:
#if CONFIG_APP_ENABLE_ADVANCED_MODE
        return APP_MODE_ADVANCED;
#else
        return APP_MODE_TEST;
#endif
#if CONFIG_APP_ENABLE_ADVANCED_MODE
    case APP_MODE_ADVANCED:
        return APP_MODE_TEST;
#endif
    default:
        return APP_MODE_TEST;
    }
}

uint32_t mode_manager_period_ms(enum app_mode mode)
{
    switch (mode) {
    case APP_MODE_TEST:
        return CONFIG_APP_TEST_PERIOD_MS;
    case APP_MODE_NORMAL:
        return CONFIG_APP_NORMAL_PERIOD_MS;
    case APP_MODE_SIMU:
        return CONFIG_APP_SIMU_PERIOD_MS;
#if CONFIG_APP_ENABLE_ADVANCED_MODE
    case APP_MODE_ADVANCED:
        return CONFIG_APP_NORMAL_PERIOD_MS;
#endif
    default:
        return CONFIG_APP_TEST_PERIOD_MS;
    }
}

enum app_led_color mode_manager_led_for(enum app_mode mode,
                                        enum tracker_state state)
{
    switch (mode) {
    case APP_MODE_TEST:
        return APP_LED_BLUE;
    case APP_MODE_SIMU:
        /* SIMU indication is independent of the tracker state. */
        return APP_LED_MAGENTA;
    case APP_MODE_NORMAL:
        switch (state) {
        case TRACKER_STATE_IDLE:
            return APP_LED_GREEN;
        case TRACKER_STATE_FLYING:
            return APP_LED_YELLOW;
        case TRACKER_STATE_PROBLEM:
            return APP_LED_RED;
        default:
            return APP_LED_RED;
        }
#if CONFIG_APP_ENABLE_ADVANCED_MODE
    case APP_MODE_ADVANCED:
        return APP_LED_WHITE;
#endif
    default:
        return APP_LED_OFF;
    }
}

const char *mode_manager_name(enum app_mode mode)
{
    switch (mode) {
    case APP_MODE_TEST:
        return "TEST";
    case APP_MODE_NORMAL:
        return "NORMAL";
    case APP_MODE_SIMU:
        return "SIMU";
#if CONFIG_APP_ENABLE_ADVANCED_MODE
    case APP_MODE_ADVANCED:
        return "ADVANCED";
#endif
    default:
        return "UNKNOWN";
    }
}

const char *mode_manager_led_name(enum app_led_color color)
{
    switch (color) {
    case APP_LED_OFF:
        return "OFF";
    case APP_LED_BLUE:
        return "BLUE";
    case APP_LED_GREEN:
        return "GREEN";
    case APP_LED_YELLOW:
        return "YELLOW";
    case APP_LED_RED:
        return "RED";
    case APP_LED_MAGENTA:
        return "MAGENTA";
    case APP_LED_WHITE:
        return "WHITE";
    default:
        return "UNKNOWN";
    }
}
