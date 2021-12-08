#pragma once

#include <stdbool.h>

#include "main.h"


void lcd_init(I2C_HandleTypeDef * i2c);
void lcd_step(I2C_HandleTypeDef * i2c);

void lcd_reset_screen();
void lcd_set_pixel(uint8_t x, uint8_t y, bool color);
void lcd_fill_rect(uint8_t ax, uint8_t ay, uint8_t bx, uint8_t by, bool color);
