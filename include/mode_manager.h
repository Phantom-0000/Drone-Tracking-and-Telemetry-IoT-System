#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "telemetry_snapshot.h"

/* The mode order is part of the mandatory B1 behaviour. */
enum app_mode {
    APP_MODE_TEST = 0,
    APP_MODE_NORMAL,
    APP_MODE_SIMU,
#if CONFIG_APP_ENABLE_ADVANCED_MODE
    APP_MODE_ADVANCED,
#endif
};

enum app_led_color {
    APP_LED_OFF = 0,
    APP_LED_BLUE,
    APP_LED_GREEN,
    APP_LED_YELLOW,
    APP_LED_RED,
    APP_LED_MAGENTA,
    APP_LED_WHITE,
};

struct mode_manager {
    enum app_mode mode;
    uint32_t button_debounce_ms;
    uint32_t last_button_ms;
    bool button_timestamp_valid;
};

void mode_manager_init(struct mode_manager *manager,
                       uint32_t button_debounce_ms);

/* Bridge for the eventual B1 GPIO callback. */
void app_b1_button_pressed(uint32_t timestamp_ms);

/* Returns true only when the press is outside the debounce interval. */
bool mode_manager_accept_button(struct mode_manager *manager,
                                uint32_t timestamp_ms);

enum app_mode mode_manager_next_mode(enum app_mode mode);
uint32_t mode_manager_period_ms(enum app_mode mode);
enum app_led_color mode_manager_led_for(enum app_mode mode,
                                        enum tracker_state state);
const char *mode_manager_name(enum app_mode mode);
const char *mode_manager_led_name(enum app_led_color color);

#endif /* MODE_MANAGER_H */
