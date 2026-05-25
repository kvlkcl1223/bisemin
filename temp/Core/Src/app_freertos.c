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
#include "tm1638_board.h"
#include "temp_panel.h"
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
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
    .name = "controlTask",
    .priority = (osPriority_t)osPriorityHigh,
    .stack_size = 512 * 4};
/* Definitions for HMITask */
osThreadId_t HMITaskHandle;
const osThreadAttr_t HMITask_attributes = {
    .name = "HMITask",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 256 * 4};
/* Definitions for Queue_Temp */
osMessageQueueId_t Queue_TempHandle;
const osMessageQueueAttr_t Queue_Temp_attributes = {
    .name = "Queue_Temp"};
/* Definitions for SysStateMutex */
osMutexId_t SysStateMutexHandle;
const osMutexAttr_t SysStateMutex_attributes = {
    .name = "SysStateMutex"};

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

  /* Create the queue(s) */
  /* creation of Queue_Temp */
  Queue_TempHandle = osMessageQueueNew(4, sizeof(float), &Queue_Temp_attributes);

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
  uint32_t now;

  /*
   * PID 控制循环 (100Hz = 10ms)
   *
   * 伪代码示意:
   *
   *   float current_temp = get_cell_temperature(cell);
   *   float output = PID_Compute(pid, current_temp);
   *   if (output > 0) { // 加热
   *       __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)output);
   *       __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
   *   } else { // 制冷
   *       __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
   *       __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(-output));
   *   }
   */
  for (;;)
  {
    // now = osKernelGetTickCount();

    // /* 对两个恒温池执行 PID 控制循环 */
    // for (uint8_t cell = 0; cell < PANEL_CELL_NUM; cell++)
    // {
    //   TempCell_t *c = &g_panel.cell[cell];

    //   if (c->run_mode != CELL_STOP && c->pid_enabled)
    //   {
    //     float current = (float)c->current_temp_x10 * 0.1f;
    //     /* PID_TypeDef *pid = get_pid_for_cell(cell); */
    //     float output = PID_Compute(&temp_pid, current);

    //     /* --- 映射到 H 桥 PWM (请根据你的实际电路修改) --- */
    //     if (cell == 0)
    //     {
    //       if (output > 0.0f)
    //       {
    //         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)output);
    //         /* 关闭制冷通道 */
    //       }
    //       else
    //       {
    //         /* 关闭加热通道 */
    //         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(-output));
    //       }
    //     }
    //     /* cell == 1: 使用 TIM8, 根据你的实际硬件修改 */
    //   }
    // }

    osDelay(10); /* 100Hz */
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
  uint32_t now;
  static uint32_t last_tick = 0;

  for (;;)
  {
    now = osKernelGetTickCount();

    /* 每 20ms 扫描按键并驱动面板逻辑 */
    if (now - last_tick >= 20)
    {
      last_tick = now;

      /* 1. 扫描 TM1638 按键 (内部触发 OnTM1638Key → TempPanel_KeyEvent) */
      TM1638_ProcessKeys(&htm1638);

      /* 2. 面板周期任务: 超时检测 + 模式切换 + 显示刷新 */
      TempPanel_Task(&g_panel, now);

      /* 3. 心跳 LED 闪烁 (每 500ms 翻转) */
      static uint32_t last_blink = 0;
      if (now - last_blink >= 500)
      {
        last_blink = now;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }
    }

    osDelay(5); /* 5ms tick, 让 FreeRTOS 有机会调度 */
  }
  /* USER CODE END StartHMITask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
