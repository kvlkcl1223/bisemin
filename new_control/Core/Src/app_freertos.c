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
#include "bridge1_test.h"

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
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
    .name = "ControlTask",
    .priority = (osPriority_t)osPriorityHigh,
    .stack_size = 512 * 4};
/* Definitions for HMITask */
osThreadId_t HMITaskHandle;
const osThreadAttr_t HMITask_attributes = {
    .name = "HMITask",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 256 * 4};
/* Definitions for SysStateMutex */
osMutexId_t SysStateMutexHandle;
const osMutexAttr_t SysStateMutex_attributes = {
    .name = "SysStateMutex"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartControlask(void *argument);
void StartHMITask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlask, NULL, &ControlTask_attributes);

  /* creation of HMITask */
  HMITaskHandle = osThreadNew(StartHMITask, NULL, &HMITask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartControlask */
/**
 * @brief  Function implementing the ControlTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlask */
void StartControlask(void *argument)
{
  /* USER CODE BEGIN StartControlask */
  /* Infinite loop */

  Bridge1_Status_t ret;

  osDelay(1000);

  ret = Bridge1_RunBasicTest();

  for (;;)
  {
    if (ret == BRIDGE1_OK)
    {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      osDelay(1000);
    }
    else
    {
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      osDelay(200);
    }
  }

  /* USER CODE END StartControlask */
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
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    osDelay(100);
  }
  /* USER CODE END StartHMITask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
