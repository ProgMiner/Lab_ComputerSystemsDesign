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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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

static void set_green_led(int on) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_yellow_led(int on) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_red_led(int on) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int is_btn_clicked(int * lastPressTime) {
	// reset = pressed

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET && HAL_GetTick() - *lastPressTime > 1000) {
		*lastPressTime = HAL_GetTick();

		return 1;
	}

	return 0;
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
/* USER CODE BEGIN 2 */

	set_green_led(0);
	set_yellow_led(0);
	set_red_led(0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i = 0, state[5] = { 0 }, lastPressTime = 0, elapsedStateTime[5] = { 0 }, t = HAL_GetTick();
  while (1) {
      if (is_btn_clicked(&lastPressTime)) {
          i = (i + 1) % 5;

        	set_green_led(0);
        	set_yellow_led(0);
        	set_red_led(0);
      }

      elapsedStateTime[i] += HAL_GetTick() - t;
      t = HAL_GetTick();

      switch (i) {
      case 0:
          switch (state[i]) {
          case 0:
              set_yellow_led(1);

              if (elapsedStateTime[i] >= 250) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 1:
              set_green_led(1);
              set_yellow_led(0);

              if (elapsedStateTime[i] >= 250) {
                    elapsedStateTime[i] = 0;
                  state[i] = 0;
              }
          }
          break;

      case 1:
      	switch(state[i]) {
      	case 0:
              set_red_led(1);

              if (elapsedStateTime[i] >= 250) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;

          case 1:
              set_red_led(0);

              if (elapsedStateTime[i] >= 250) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;

          case 2:
              set_yellow_led(1);

              if (elapsedStateTime[i] >= 250) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;

          case 3:
            	set_yellow_led(0);

              if (elapsedStateTime[i] >= 250) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 4:
              set_green_led(1);

              if (elapsedStateTime[i] >= 250) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 5:
              set_green_led(0);

              if (elapsedStateTime[i] >= 250) {
                  elapsedStateTime[i] = 0;
                  state[i] = 0;
              }
          }
          break;

      case 2:
          switch(state[i]) {
          case 0:
              set_green_led(1);

              if (elapsedStateTime[i] >= 400) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 1:
              set_green_led(0);

              if (elapsedStateTime[i] >= 250) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 2:
              set_red_led(1);

              if (elapsedStateTime[i] >= 400) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 3:
              set_red_led(0);

              if (elapsedStateTime[i] >= 250) {
                  elapsedStateTime[i] = 0;
                  state[i] = 0;
              }
          }
          break;
      case 3:
          switch(state[i]) {
          case 0:
              set_red_led(1);

              if (elapsedStateTime[i] >= 3000) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 1:
              set_red_led(0);

              if (elapsedStateTime[i] >= 1000) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 2:
              set_yellow_led(1);

              if (elapsedStateTime[i] >= 3000) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 3:
              set_yellow_led(0);

              if (elapsedStateTime[i] >= 1000) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 4:
              set_green_led(1);

              if (elapsedStateTime[i] >= 3000) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 5:
              set_green_led(0);
              if (elapsedStateTime[i] >= 1000) {
                  elapsedStateTime[i] = 0;
                  state[i] = 0;
              }
          }
          break;

      case 4:
          switch(state[i]) {
          case 0:
              set_red_led(1);

              if (elapsedStateTime[i] >= 1500) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 1:
              set_red_led(0);
              set_yellow_led(1);

              if (elapsedStateTime[i] >= 1500) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 2:
            	set_yellow_led(0);

              if (elapsedStateTime[i] >= 1000) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 3:
              set_green_led(1);

              if (elapsedStateTime[i] >= 3000) {
                  elapsedStateTime[i] = 0;
                  ++state[i];
              }
              break;
          case 4:
              set_green_led(0);
              if (elapsedStateTime[i] >= 1000) {
                  elapsedStateTime[i] = 0;
                  state[i] = 0;
              }
          }
          break;
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
