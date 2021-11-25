#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "main.h"



#define KB_EVENT_KEY_1 (0x1)
#define KB_EVENT_KEY_2 (0x2)
#define KB_EVENT_KEY_3 (0x4)
#define KB_EVENT_KEY_4 (0x8)
#define KB_EVENT_KEY_5 (0x10)
#define KB_EVENT_KEY_6 (0x20)
#define KB_EVENT_KEY_7 (0x40)
#define KB_EVENT_KEY_8 (0x80)
#define KB_EVENT_KEY_9 (0x100)
#define KB_EVENT_KEY_10 (0x200)
#define KB_EVENT_KEY_11 (0x400)
#define KB_EVENT_KEY_12 (0x800)


typedef uint16_t kb_event_key;

struct kb_event {
	kb_event_key key;
};


bool kb_event_has();
struct kb_event kb_event_pop();

void kb_init(I2C_HandleTypeDef * i2c);
void kb_scan_step(I2C_HandleTypeDef * i2c);
