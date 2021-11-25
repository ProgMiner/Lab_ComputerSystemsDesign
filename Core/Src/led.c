#include "led.h"

#include "main.h"


extern TIM_HandleTypeDef htim4;

const char * const led_names[] = {
    [LED_GREEN] = "GREEN",
    [LED_YELLOW] = "YELLOW",
    [LED_RED] = "RED",
};


typedef void (* led_set_function)(uint16_t);


static void led_green_set_function(uint16_t power) {
    htim4.Instance->CCR2 = power;
}

static void led_yellow_set_function(uint16_t power) {
    htim4.Instance->CCR3 = power;
}

static void led_red_set_function(uint16_t power) {
    htim4.Instance->CCR4 = power;
}

static const led_set_function led_set_functions[] = {
    [LED_GREEN] = led_green_set_function,
    [LED_YELLOW] = led_yellow_set_function,
    [LED_RED] = led_red_set_function,
};

void led_set_power(enum led led, uint8_t power) {
    if (power > 100) {
        power = 100;
    }

    led_set_functions[led]((uint16_t) power * 10);
}
