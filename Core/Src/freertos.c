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
#include "adc.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MEDIAN_SIZE 11  // 建议奇数（3/5/7）
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
/* Definitions for Task_LED0 */
osThreadId_t Task_LED0Handle;
const osThreadAttr_t Task_LED0_attributes = {
  .name = "Task_LED0",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_LED1 */
osThreadId_t Task_LED1Handle;
const osThreadAttr_t Task_LED1_attributes = {
  .name = "Task_LED1",
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
/* Definitions for Task_USART */
osThreadId_t Task_USARTHandle;
const osThreadAttr_t Task_USART_attributes = {
  .name = "Task_USART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Start_led1_Task(void *argument);
void Start_adc_Task(void *argument);
void Start_usart_Task(void *argument);

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
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_LED0 */
  Task_LED0Handle = osThreadNew(Start_led1_Task, NULL, &Task_LED0_attributes);

  /* creation of Task_LED1 */
  Task_LED1Handle = osThreadNew(Start_led1_Task, NULL, &Task_LED1_attributes);

  /* creation of Task_ADC */
  Task_ADCHandle = osThreadNew(Start_adc_Task, NULL, &Task_ADC_attributes);

  /* creation of Task_USART */
  Task_USARTHandle = osThreadNew(Start_usart_Task, NULL, &Task_USART_attributes);

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

/* USER CODE BEGIN Header_Start_led1_Task */
/**
* @brief Function implementing the Task_LED0 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_led1_Task */
void Start_led1_Task(void *argument)
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
* @brief Function implementing the Task_ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_adc_Task */
void Start_adc_Task(void *argument)
{
  /* USER CODE BEGIN Start_adc_Task */
  float adc_v = 0.f;
	float current = 0.f;
	float z_adc_v = 0.f;
	float z_current = 0.f;
		float adc_buffer[MEDIAN_SIZE] = {0};
    uint8_t adc_index = 0;
    float tempBuffer[MEDIAN_SIZE];
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
		current = (adc_v - 2.475)/0.066 ;
		
		snprintf(buffer, 20, "V: %.2f\t", adc_v);
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, 20, 10);
		snprintf(buffer, 20, "I: %.2f\t", current);
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, 20, 10);
		
		adc_buffer[adc_index++] = adc_v;
    adc_index %= MEDIAN_SIZE;
    
    // 复制并排序
    memcpy(tempBuffer, adc_buffer, sizeof(adc_buffer));
    for (int i = 0; i < MEDIAN_SIZE - 1; i++) {
        for (int j = i + 1; j < MEDIAN_SIZE; j++) {
            if (tempBuffer[i] > tempBuffer[j]) {
                float temp = tempBuffer[i];
                tempBuffer[i] = tempBuffer[j];
                tempBuffer[j] = temp;
            }
        }
    }
		
		if(adc_index >= MEDIAN_SIZE)
		{
			z_adc_v = tempBuffer[(MEDIAN_SIZE+1) / 2];  // 返回中位数
			z_current = (z_adc_v - 2.475)/0.066 ;
		}
		
		snprintf(buffer, 20, "zV:\t%.2f\t", z_adc_v);
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, 20, 10);
		snprintf(buffer, 20, "zI:\t%.2f\n", z_current);
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, 20, 10);
    osDelay(10);
  }
  /* USER CODE END Start_adc_Task */
}

/* USER CODE BEGIN Header_Start_usart_Task */
/**
* @brief Function implementing the Task_USART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_usart_Task */
void Start_usart_Task(void *argument)
{
  /* USER CODE BEGIN Start_usart_Task */
  uint8_t Txdata[20] = "transmit !!!\n";
  /* Infinite loop */
  for(;;)
  {
		//HAL_UART_Transmit(&huart1, Txdata, 20, 10);
    osDelay(1);
  }
  /* USER CODE END Start_usart_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

