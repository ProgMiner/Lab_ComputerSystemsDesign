#pragma once

#include <stdint.h>
#include <string.h>


void uart_send_message(const char * content, uint32_t length);
void uart_send_message_string_nl(const char * message);
void uart_send_message_format(const char * format, ...);

static inline void uart_send_message_string(const char * message) {
	uart_send_message(message, strlen(message));
}
