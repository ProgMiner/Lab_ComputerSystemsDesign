#include "kb.h"

#include <stdbool.h>
#include <stddef.h>

#include "uart.h"


#define BUFFER_CAPACITY (32)
#define INC_BUFFER_IDX(__idx) do { __idx = (__idx + 1) % BUFFER_CAPACITY; } while (0)


static struct kb_event buffer[BUFFER_CAPACITY] = { 0 };
static size_t buffer_start_idx = 0;
static size_t buffer_end_idx = 0;


static void kb_event_push(struct kb_event event) {
	buffer[buffer_end_idx] = event;

	INC_BUFFER_IDX(buffer_end_idx);
}

bool kb_event_has() {
	return buffer_start_idx != buffer_end_idx;
}

struct kb_event kb_event_pop() {
	const struct kb_event evt = buffer[buffer_start_idx];

	INC_BUFFER_IDX(buffer_start_idx);
	return evt;
}

void kb_init(I2C_HandleTypeDef * i2c) {
	uint8_t config = 0x70;

	HAL_I2C_Mem_Write(i2c, 0xE2, 3, 1, &config, 1, 100);
}

static inline void kb_write_output(I2C_HandleTypeDef * i2c, uint8_t data) {
	HAL_I2C_Mem_Write_IT(i2c, 0xE2, 1, 1, &data, 1);
}

static inline void kb_read_input(I2C_HandleTypeDef * i2c, uint8_t * data) {
	HAL_I2C_Mem_Read_IT(i2c, 0xE3, 1, 1, data, 1);
}

void kb_scan_step(I2C_HandleTypeDef * i2c) {
	static uint8_t reg_buffer = ~0;

	static int row = 0;
	static bool read = false;
	static kb_event_key keys = 0;

	if (HAL_I2C_GetState(i2c) != HAL_I2C_STATE_READY) {
		// uart_send_message_format("%d keyboard is busy\n", HAL_GetTick());
		return;
	}

	if (!read) {
		// handle read data
		for (int i = 0, mask = 0x10; i < 3; ++i, mask <<= 1) {
			if ((reg_buffer & mask) == 0) {
				keys |= 0x1 << (row * 3 + i);
			}
		}

		// move to next row
		row = (row + 1) % 4;

		// if read all rows - handle results
		if (row == 0) {
			int count = 0;
			for (int i = 0, key = 0x1; i < 12; ++i, key <<= 1) {
				if ((keys & key) != 0) {
					++count;
				}
			}

			if (count <= 2) {
				for (int i = 0, key = 0x1; i < 12; ++i, key <<= 1) {
					if ((keys & key) != 0) {
						kb_event_push((struct kb_event) { .key = key });
					}
				}
			}

			keys = 0;
		}

		// start write
		kb_write_output(i2c, ~((uint8_t) (1 << row)));
	} else {
		// start read
		kb_read_input(i2c, &reg_buffer);
	}

	read = !read;
}
