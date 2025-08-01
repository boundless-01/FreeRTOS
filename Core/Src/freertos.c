/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "queue.h"
#include "stdio.h"
#include "adc.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	float adc_v = 0.f;
	float current = 0.f;
	float z_adc_v = 0.f;
	float z_current = 0.f;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_LED1 */
osThreadId_t Task_LED1Handle;
const osThreadAttr_t Task_LED1_attributes = {
  .name = "Task_LED1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_LED2 */
osThreadId_t Task_LED2Handle;
const osThreadAttr_t Task_LED2_attributes = {
  .name = "Task_LED2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_Uart */
osThreadId_t Task_UartHandle;
const osThreadAttr_t Task_Uart_attributes = {
  .name = "Task_Uart",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_ADC */
osThreadId_t Task_ADCHandle;
const osThreadAttr_t Task_ADC_attributes = {
  .name = "Task_ADC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for msgQueue */
osMessageQueueId_t msgQueueHandle;
const osMessageQueueAttr_t msgQueue_attributes = {
  .name = "msgQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of msgQueue */
  msgQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &msgQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_LED1 */
  Task_LED1Handle = osThreadNew(StartTask02, NULL, &Task_LED1_attributes);

  /* creation of Task_LED2 */
  Task_LED2Handle = osThreadNew(StartTask03, NULL, &Task_LED2_attributes);

  /* creation of Task_Uart */
  Task_UartHandle = osThreadNew(StartTask04, NULL, &Task_Uart_attributes);

  /* creation of Task_ADC */
  Task_ADCHandle = osThreadNew(StartTask05, NULL, &Task_ADC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task_LED1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	static uint16_t sendValue = 0;
  /* Infinite loop */
  for(;;)
  {
//		if(led1time >= 100)
//		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
			sendValue++;
			//led1time = 0;
			xQueueSend(msgQueueHandle, &sendValue, 100);
			osDelay(100);
//		}
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task_LED2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	static uint8_t valueTrue = 0;
	static uint16_t receiveValue = 0;
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(msgQueueHandle, &receiveValue, 100);
		if(receiveValue == 1)	valueTrue = 1;
		if(valueTrue == 0)
		{
//			if(led2time >= 500)
//			{
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
				led2time = 0;
				osDelay(2000);
//			}
		}
		else
		{
//			if(led2time >= 200)
//			{
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
				led2time = 0;
				osDelay(100);
//			}
		}
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Task_Uart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	uint8_t TxData[20] = "usart1 transmit!\n";
	char buffer[100] = {0};
	snprintf(buffer, sizeof(buffer), "电压:%.3f\t电流:%.3f\t电压:%.3f\n", adc, current, z_current);
  /* Infinite loop */
  for(;;)
  {
		HAL_UART_Transmit(&huart1, (uint8_t*)TxData, 20, 10);
    osDelay(10);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Task_ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
	static float buffer[11] = {0};
	static float adc_buffer[11] = {0};
	static uint8_t index = 0;
  /* Infinite loop */
  for(;;)
  {
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 30) == HAL_OK)
		{
			adc_v = HAL_ADC_GetValue(&hadc1) / 4095.f * 3.3;
			current = (z_adc_v - 2.475) / 0.066;
		}
		HAL_ADC_Stop(&hadc1);
		buffer[index++] = adc_v;
		index %= 11;
		
		memcpy(adc_buffer, buffer, sizeof(buffer));
		for(int i=0; i<10; i++)
		{
			for(int k=0; k<11; k++)
			{
				if(adc_buffer[k+1] > adc_buffer[k])
				{
					float temp = adc_buffer[k];
					adc_buffer[k] = adc_buffer[k+1];
					adc_buffer[k+1] = temp;
				}
			}
		}
		
		z_adc_v = adc_buffer[6];
		z_current = (z_adc_v - 2.475) / 0.066;
    osDelay(50);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

