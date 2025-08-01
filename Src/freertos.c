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
#include "adc.h"
#include "stdio.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId TaskLed1Handle;
osThreadId TaskAdcHandle;
osThreadId TaskUartHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start_led1_Task(void const * argument);
void Start_adc_Task(void const * argument);
void Start_uart_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TaskLed1 */
  osThreadDef(TaskLed1, Start_led1_Task, osPriorityIdle, 0, 128);
  TaskLed1Handle = osThreadCreate(osThread(TaskLed1), NULL);

  /* definition and creation of TaskAdc */
  osThreadDef(TaskAdc, Start_adc_Task, osPriorityIdle, 0, 128);
  TaskAdcHandle = osThreadCreate(osThread(TaskAdc), NULL);

  /* definition and creation of TaskUart */
  osThreadDef(TaskUart, Start_uart_Task, osPriorityIdle, 0, 128);
  TaskUartHandle = osThreadCreate(osThread(TaskUart), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_led1_Task */
/**
* @brief Function implementing the TaskLed1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_led1_Task */
void Start_led1_Task(void const * argument)
{
  /* USER CODE BEGIN Start_led1_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_led1_Task */
}

/* USER CODE BEGIN Header_Start_adc_Task */
/**
* @brief Function implementing the TaskAdc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_adc_Task */
void Start_adc_Task(void const * argument)
{
  /* USER CODE BEGIN Start_adc_Task */
	float adc_v = 0.f;
	char buffer[20] = {0};
  /* Infinite loop */
  for(;;)
  {
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion (&hadc1 , 30) == HAL_OK)
		{
			adc_v = HAL_ADC_GetValue(&hadc1) / 4095.f * 3.3;
		}
		HAL_ADC_Start(&hadc1);
		snprintf(buffer, 20, "V:\t%.2f\n", adc_v);
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, 20, 10);
    osDelay(10);
  }
  /* USER CODE END Start_adc_Task */
}

/* USER CODE BEGIN Header_Start_uart_Task */
/**
* @brief Function implementing the TaskUart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_uart_Task */
void Start_uart_Task(void const * argument)
{
  /* USER CODE BEGIN Start_uart_Task */
	uint8_t Txdata[20] = "transmit !!!\n";
  /* Infinite loop */
  for(;;)
  {
		//HAL_UART_Transmit(&huart1, Txdata, 20, 10);
    osDelay(1);
  }
  /* USER CODE END Start_uart_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

