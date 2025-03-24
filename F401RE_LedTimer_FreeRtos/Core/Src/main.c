/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "string.h"
#include "stdlib.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef uint8_t		bool;
typedef uint32_t	blinkInterval;

typedef struct {
	uint8_t		str[25];
	uint8_t 	size;
}	queueItem_t;

typedef struct {
	bool			isOn;
	blinkInterval	blinkInt;

} ledStatus_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define QUEUE_SIZE 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**************** QUEUE HANDLER *****************/
xQueueHandle	rxQueueHandler;


/**************** TASK HANDLER *****************/
xTaskHandle	 	rxUartData;
xTaskHandle		ledControl;
xTaskHandle		parseData;

/*************** TASK FUNCTIONS ****************/
void			rxUartDataTask(void* argument);
void			ledControlTask(void* argument);
void			parseDataTask(void* argument);

ledStatus_t	ledStatus;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  ledStatus.isOn		= 0;
  ledStatus.blinkInt	= 1000;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /*		Queue create 		*/
  if ((rxQueueHandler = xQueueCreate(QUEUE_SIZE, sizeof(queueItem_t))))
  {
	 char* str = "Queue are created:\n\r";
	 HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 20);
  }
  else
  {
	 char* str = "Can't create Queue:\n\r";
	 HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 20);
  }

  /*		Task create 		*/
  xTaskCreate(rxUartDataTask, "rxUart", 128, NULL, 1, &rxUartData);
  xTaskCreate(ledControlTask, "ledCtr", 128, NULL, 1, &ledControl);
  xTaskCreate(parseDataTask, "parseData", 128, NULL, 2, &parseData);


//  HAL_UART_Receive_IT(&huart2, &rxData, 1);
  /*		Task scheduler start 		*/
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void			ledControlTask(void* argument)
{
	for (;;)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		vTaskDelay(ledStatus.blinkInt);
	}
}

void			parseDataTask(void* argument)
{
	for (;;)
	{
		vTaskDelay(3000);
	}
}

void	rxUartDataTask(void* argument)
{
	uint8_t sym;
	uint8_t itter;
	uint8_t buff[25];

	itter	= 0;
	sym 	= '\0';
	memset(buff, '\0', 25);

	char* str0 = "rxUartDataTask in:\n\r";
	char* str1 = "------------------\n\r";
	HAL_UART_Transmit(&huart2, (uint8_t*)str0, strlen(str0), 20);
	HAL_UART_Transmit(&huart2, (uint8_t*)str1, strlen(str1), 20);


	for (;;)
	{
		while (HAL_UART_Receive(&huart2, &sym, sizeof(uint8_t), 500) == HAL_OK && ((sym != '\n') || (sym != '\0')))
		{
			if (itter < 20 && sym != '\0' && sym != '\r')
				buff[itter++] = sym;
		}
		if (itter > 0)
		{
			queueItem_t qElem;

			qElem.size = itter;
			memset(qElem.str, '\0', 25);
			memcpy(qElem.str, buff, itter);
			if (xQueueSend(rxQueueHandler, &qElem, 20) == pdPASS)
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)str1, strlen(str1), 20);
				char *str2 = "Successfully sent\n\r";
				HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), 20);
				HAL_UART_Transmit(&huart2, (uint8_t*)str1, strlen(str1), 20);
			}
			itter	= 0;
			memset(buff, '\0', 25);
		}
		vTaskDelay(3000);
	}

}

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
