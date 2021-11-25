/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum led {
    LED_NO_ONE = 0,
    LED_RED = 1,
    LED_GREEN = 2,
    LED_YELLOW = 3,
};

struct led_state {
    enum led led;
    uint32_t timeout;
};

struct mode {
    uint8_t states_n;
    struct led_state states[8];
};

struct modes {
    uint8_t modes_n;
    struct mode modes[8];
};

struct state {
	uint8_t modes_n;
	struct mode modes[8];
	uint8_t prev_changed_mode;
    uint8_t i;
    uint8_t state[8];
    uint32_t elapsedStateTime[8];
    bool use_interrupt;
    char command_line[256];
    struct mode new_mode;
    uint8_t await_n_timeouts;
};

typedef void (* set_led_function)(bool);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TIMEOUT 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void set_green_led(bool on) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_yellow_led(bool on) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_red_led(bool on) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int is_btn_clicked(uint32_t * lastPressTime) {
	// reset = pressed

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET && HAL_GetTick() - *lastPressTime > 1000) {
		*lastPressTime = HAL_GetTick();

		return 1;
	}

	return 0;
}

static void set_no_one_led(bool on) {
    // do nothing
    (void) on;
}

static const set_led_function led_functions[] = {
    [LED_NO_ONE] = set_no_one_led,
    [LED_RED] = set_red_led,
    [LED_GREEN] = set_green_led,
    [LED_YELLOW] = set_yellow_led,
};

const struct modes default_modes = {
    .modes_n = 4,
    .modes = {
        {
            .states_n = 2,
            .states = {
                {
                    .led = LED_YELLOW,
                    .timeout = 250,
                },
                {
                    .led = LED_GREEN,
                    .timeout = 250,
                },
            },
        },
        {
            .states_n = 6,
            .states = {
                {
                    .led = LED_RED,
                    .timeout = 250,
                },
                {
                    .led = LED_NO_ONE,
                    .timeout = 250,
                },
                {
                    .led = LED_YELLOW,
                    .timeout = 250,
                },
                {
                    .led = LED_NO_ONE,
                    .timeout = 250,
                },
                {
                    .led = LED_GREEN,
                    .timeout = 250,
                },
                {
                    .led = LED_NO_ONE,
                    .timeout = 250,
                },
            },
        },
        {
            .states_n = 4,
            .states = {
                {
                    .led = LED_GREEN,
                    .timeout = 400,
                },
                {
                    .led = LED_NO_ONE,
                    .timeout = 250,
                },
                {
                    .led = LED_RED,
                    .timeout = 400,
                },
                {
                    .led = LED_NO_ONE,
                    .timeout = 250,
                },
            },
        },
        {
            .states_n = 6,
            .states = {
                {
                    .led = LED_RED,
                    .timeout = 3000,
                },
                {
                    .led = LED_NO_ONE,
                    .timeout = 1000,
                },
                {
                    .led = LED_YELLOW,
                    .timeout = 3000,
                },
                {
                    .led = LED_NO_ONE,
                    .timeout = 1000,
                },
                {
                    .led = LED_GREEN,
                    .timeout = 3000,
                },
                {
                    .led = LED_NO_ONE,
                    .timeout = 1000,
                },
            },
        },
    },
};

static char received_char;
static bool is_char_received = false;
static bool is_transmitted = true;

static void send_uart_message(struct state * state, const char * content, uint32_t length) {
	if (state->use_interrupt) {
		while (!is_transmitted);

		is_transmitted = false;
		HAL_UART_Transmit_IT(&huart6, (void *) content, length);
	} else {
		HAL_UART_Transmit(&huart6, (void *) content, length, UART_TIMEOUT);
	}
}

static inline void send_uart_message_string(struct state * state, const char * message) {
	send_uart_message(state, message, strlen(message));
}

static void send_uart_message_string_nl(struct state * state, const char * message) {
	send_uart_message_string(state, message);
	send_uart_message(state, "\r\n", 2);
}

static void send_uart_message_format(struct state * state, const char * format, ...) {
	static char buffer[1024];

	if (state->use_interrupt) {
		while (!is_transmitted);
	}

	va_list ap;
	va_start(ap, format);
	vsnprintf(buffer, sizeof(buffer), format, ap);
	va_end(ap);

	send_uart_message_string(state, buffer);
}

static bool string_equals(const char * a, const char * b) {
	return strcmp(a, b) == 0;
}

static bool starts_with(const char * prefix, const char * str) {
	return strncmp(prefix, str, strlen(prefix)) == 0;
}

static void normalize_command_line(char * command_line) {
	uint32_t length = strlen(command_line);

	uint32_t j = 0;
	bool prev_space = true;
	for (uint32_t i = 0; i < length; ++i) {
		if (isspace(command_line[i])) {
			if (prev_space) {
				continue;
			} else {
				prev_space = true;
				command_line[i] = ' ';
			}
		} else {
			prev_space = false;
		}

		command_line[j++] = command_line[i];
	}

	if (prev_space) {
		while (isspace(command_line[j - 1])) {
			--j;
		}
	}

	command_line[j] = '\0';
}

static void set_active_mode(struct state * state, uint8_t idx) {
	led_functions[state->modes[state->i].states[state->state[state->i]].led](false);

	state->i = idx;

	if (state->modes[state->i].states_n > 0) {
		led_functions[state->modes[state->i].states[state->state[state->i]].led](true);
	}
}

static bool handle_set_command(struct state * state) {
	const char * const mode_idx_str = state->command_line + 4;
	uint32_t mode_idx;

	if (sscanf(mode_idx_str, "%lu", &mode_idx) != 1) {
		return false;
	}

	if (mode_idx < 1 || mode_idx > state->modes_n) {
		return false;
	}

	set_active_mode(state, mode_idx - 1);
	return true;
}

static bool handle_new_command(struct state * state) {
	const char * const pattern = state->command_line + 4;
	const uint32_t pattern_length = strlen(pattern);

	if (pattern_length < 2 || pattern_length > 8) {
		return false;
	}

	state->new_mode.states_n = pattern_length;

	for (uint8_t i = 0; i < pattern_length; ++i) {
		enum led led;

		switch (pattern[i]) {
			case 'n':
				led = LED_NO_ONE;
				break;

			case 'r':
				led = LED_RED;
				break;

			case 'g':
				led = LED_GREEN;
				break;

			case 'y':
				led = LED_YELLOW;
				break;

			default:
				return false;
		}

		state->new_mode.states[i].led = led;
	}

	state->await_n_timeouts = pattern_length;
	send_uart_message_format(state, "Send %d timeouts in milliseconds as lines:\r\n", pattern_length);
	return true;
}

static bool handle_new_command_timeout(struct state * state) {
	const uint8_t state_idx = state->new_mode.states_n - state->await_n_timeouts;

	if (sscanf(state->command_line, "%lu", &(state->new_mode.states[state_idx].timeout)) != 1) {
		return false;
	}

	--state->await_n_timeouts;
	if (state->await_n_timeouts == 0) {
		const uint8_t available_mode_idxs = sizeof(state->modes) / sizeof(*(state->modes)) - default_modes.modes_n;
		const uint8_t mode_idx = default_modes.modes_n + (state->prev_changed_mode - default_modes.modes_n + 1) % available_mode_idxs;

		if (state->modes_n <= mode_idx) {
			state->modes_n = mode_idx + 1;
		}

		state->prev_changed_mode = mode_idx;
		memcpy(state->modes + mode_idx, &(state->new_mode), sizeof(state->new_mode));

		send_uart_message_format(state, "Written in mode %d\r\n", mode_idx + 1);
		return true;
	}

	send_uart_message_format(state, "%d timeouts remaining:\r\n", state->await_n_timeouts);
	return true;
}

static void handle_command_line(struct state * state) {
	normalize_command_line(state->command_line);

	if (strlen(state->command_line) == 0) {
		return;
	}

	if (state->await_n_timeouts > 0) {
		if (handle_new_command_timeout(state)) {
			return;
		}
	}

	if (string_equals("set interrupts on", state->command_line)) {
		send_uart_message_string_nl(state, "Interrupts turned on");
		state->use_interrupt = true;
		return;
	}

	if (string_equals("set interrupts off", state->command_line)) {
		send_uart_message_string_nl(state, "Interrupts turned off");
		state->use_interrupt = false;
		return;
	}

	if (starts_with("set ", state->command_line)) {
		if (handle_set_command(state)) {
			return;
		}
	}

	if (starts_with("new ", state->command_line)) {
		if (handle_new_command(state)) {
			return;
		}
	}

	send_uart_message_string_nl(state, "Invalid command");
}

static void handle_uart(struct state * state) {
    if (state->use_interrupt) {
        if (!is_char_received) {
            HAL_UART_Receive_IT(&huart6, (void *) &received_char, sizeof(received_char));
            return;
        }
    } else {
        switch (HAL_UART_Receive(&huart6, (void *) &received_char, sizeof(received_char), UART_TIMEOUT)) {
            case HAL_OK:
                break;

            case HAL_ERROR:
            case HAL_BUSY:
            case HAL_TIMEOUT:
                return;
        }
    }

	is_char_received = false;

    // echo
    send_uart_message(state, &received_char, 1);

    switch (received_char) {
    	case '\b':
    	case 0x7F: {
    	    const uint32_t command_line_length = strlen(state->command_line);

    	    if (command_line_length > 0) {
    	    	state->command_line[command_line_length - 1] = '\0';
    	    }

    		return;
    	}

    	case '\r':
    	    send_uart_message_string(state, "\n");

    		handle_command_line(state);
        	memset(state->command_line, '\0', sizeof(state->command_line));
    		return;
    }

    const uint32_t command_line_length = strlen(state->command_line);

    // overflow
    if (command_line_length == sizeof(state->command_line) - 1) {
    	send_uart_message_string_nl(state, "\r\nInvalid command");
    	memset(state->command_line, '\0', sizeof(state->command_line));
    	return;
    }

    state->command_line[command_line_length] = received_char;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    (void) huart;

    is_char_received = true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    (void) huart;

    is_transmitted = true;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  	set_green_led(false);
  	set_yellow_led(false);
  	set_red_led(false);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    struct state state = {
		.modes_n = default_modes.modes_n,
		.prev_changed_mode = 7,
        .i = 0,
        .state = { 0 },
        .elapsedStateTime = { 0 },
        .use_interrupt = false,
		.command_line = { 0 },
		.new_mode = { 0 },
		.await_n_timeouts = 0,
    };

    memcpy(state.modes, default_modes.modes, sizeof(state.modes));

    if (state.modes_n == 0) {
        return 0;
    }

    if (state.modes[0].states_n > 0) {
        led_functions[state.modes[0].states[0].led](true);
    }

    uint32_t lastPressTime = 0;
    uint32_t t = HAL_GetTick();
    while (1) {
        if (is_btn_clicked(&lastPressTime)) {
            set_active_mode(&state, (state.i + 1) % state.modes_n);
        }

        handle_uart(&state);

        state.elapsedStateTime[state.i] += HAL_GetTick() - t;
        t = HAL_GetTick();

        // assume that i and state[i] is always valid
        const struct mode * current_mode = state.modes + state.i;
        if (current_mode->states_n == 0) {
            continue;
        }

        const struct led_state * current_state = current_mode->states + state.state[state.i];

        if (state.elapsedStateTime[state.i] >= current_state->timeout) {
            led_functions[current_state->led](false);

            state.elapsedStateTime[state.i] = 0;
            state.state[state.i] = (state.state[state.i] + 1) % current_mode->states_n;

            led_functions[current_mode->states[state.state[state.i]].led](true);
        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
