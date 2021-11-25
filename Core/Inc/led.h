#pragma once

#include <stdint.h>


enum led {
    LED_GREEN = 0,
    LED_YELLOW = 1,
    LED_RED = 2,
};

struct led_mode {
    enum led led;
    uint8_t power;
};

extern const char * const led_names[];


void led_set_power(enum led led, uint8_t power);

static inline void led_mode_enable(struct led_mode mode) {
    led_set_power(mode.led, mode.power);
}

static inline void led_mode_disable(struct led_mode mode) {
    led_set_power(mode.led, 0);
}
