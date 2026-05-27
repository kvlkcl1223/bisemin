/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "bridge1_test.h"
#include "opamp.h"
#include "adc.h"
#include "dac.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "stdbool.h"
#include "tim.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define BRIDGE1_NEXT_TEST_MODE 2

#define BRIDGE1_TEST_VREF_MV 1000U
#define BRIDGE1_TEST_DUTY_FORWARD 0.20f
#define BRIDGE1_TEST_DUTY_REVERSE -0.40f
/* USER CODE END PD */

/* USER CODE BEGIN Variables */
volatile uint8_t g_app_bridge1_selected_mode = BRIDGE1_NEXT_TEST_MODE;
volatile Bridge1_Status_t g_app_bridge1_start_result = BRIDGE1_PARAM_ERROR;
volatile uint8_t g_app_bridge1_loop_counter = 0;
/* USER CODE END Variables */

osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
    .name = "ControlTask",
    .priority = (osPriority_t)osPriorityHigh,
    .stack_size = 1024 * 4};

osThreadId_t HMITaskHandle;
const osThreadAttr_t HMITask_attributes = {
    .name = "HMITask",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 256 * 4};

osMutexId_t SysStateMutexHandle;
const osMutexAttr_t SysStateMutex_attributes = {
    .name = "SysStateMutex"};

void StartControlask(void *argument);
void StartHMITask(void *argument);

void MX_FREERTOS_Init(void)
{
  SysStateMutexHandle = osMutexNew(&SysStateMutex_attributes);
  ControlTaskHandle = osThreadNew(StartControlask, NULL, &ControlTask_attributes);
  HMITaskHandle = osThreadNew(StartHMITask, NULL, &HMITask_attributes);
}

void StartControlask(void *argument)
{
  /* USER CODE BEGIN StartControlask */

  osDelay(1000);

#if (BRIDGE1_NEXT_TEST_MODE == 0)
  g_app_bridge1_start_result = Bridge1_StaticMosCheck();

#elif (BRIDGE1_NEXT_TEST_MODE == 1)
  g_app_bridge1_start_result =
      Bridge1_StartMosPwmHoldTest(BRIDGE1_TEST_DUTY_FORWARD, BRIDGE1_TEST_VREF_MV);

#elif (BRIDGE1_NEXT_TEST_MODE == 2)
  g_app_bridge1_start_result =
      Bridge1_StartMosPwmHoldTest(BRIDGE1_TEST_DUTY_REVERSE, BRIDGE1_TEST_VREF_MV);

#else
#error "Invalid BRIDGE1_NEXT_TEST_MODE"
#endif

  for (;;)
  {
    Bridge1_MonitorDuringPwm();
    g_app_bridge1_loop_counter++;
    osDelay(200);
  }

  /* USER CODE END StartControlask */
}

void StartHMITask(void *argument)
{
  /* USER CODE BEGIN StartHMITask */
  for (;;)
  {
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    osDelay(100);
  }
  /* USER CODE END StartHMITask */
}

/* USER CODE BEGIN Application */
/* USER CODE END Application */
