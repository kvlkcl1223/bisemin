/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "sys_state.h"
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
// 1. 真正定义这个全局变量，并赋初值
SystemState_t g_SysState = {
    .current_temp = 25.0f,
    .target_temp = 25.0f,
    .work_mode = 0};
/* USER CODE END Variables */
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for HMITask */
osThreadId_t HMITaskHandle;
const osThreadAttr_t HMITask_attributes = {
  .name = "HMITask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for Queue_Temp */
osMessageQueueId_t Queue_TempHandle;
const osMessageQueueAttr_t Queue_Temp_attributes = {
  .name = "Queue_Temp"
};
/* Definitions for SysStateMutex */
osMutexId_t SysStateMutexHandle;
const osMutexAttr_t SysStateMutex_attributes = {
  .name = "SysStateMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartcontrolTask(void *argument);
void StartHMITask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of SysStateMutex */
  SysStateMutexHandle = osMutexNew(&SysStateMutex_attributes);

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
  /* creation of Queue_Temp */
  Queue_TempHandle = osMessageQueueNew (4, sizeof(float), &Queue_Temp_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of controlTask */
  controlTaskHandle = osThreadNew(StartcontrolTask, NULL, &controlTask_attributes);

  /* creation of HMITask */
  HMITaskHandle = osThreadNew(StartHMITask, NULL, &HMITask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartcontrolTask */
/**
 * @brief  Function implementing the controlTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartcontrolTask */
void StartcontrolTask(void *argument)
{
  /* USER CODE BEGIN StartcontrolTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartcontrolTask */
}

/* USER CODE BEGIN Header_StartHMITask */
/**
 * @brief Function implementing the HMITask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartHMITask */
void StartHMITask(void *argument)
{
  /* USER CODE BEGIN StartHMITask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHMITask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

