#include "uart.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>

#include "main.h"


extern UART_HandleTypeDef huart6;


static inline bool uart_is_ready() {
	return HAL_UART_GetState(&huart6) == HAL_UART_STATE_READY;
}

void uart_send_message(const char * content, uint32_t length) {
	while (!uart_is_ready());

	HAL_UART_Transmit_IT(&huart6, (void *) content, length);
}

void uart_send_message_string_nl(const char * message) {
	uart_send_message_string(message);
	uart_send_message("\r\n", 2);
}

void uart_send_message_format(const char * format, ...) {
	static char buffer[1024];

	while (!uart_is_ready());

	va_list ap;
	va_start(ap, format);
	vsnprintf(buffer, sizeof(buffer), format, ap);
	va_end(ap);

	uart_send_message_string(buffer);
}
