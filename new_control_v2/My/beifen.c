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
#include "drv8703_board.h"
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
#include "temp_panel.h"
#include "tm1638_board.h"
#include "sys_state.h"
/* USER CODE END Includes */
void ADC1_Test_Init(void);
void ADC1_Test_CalibrateCurrentZero(uint16_t samples);
void ADC1_Test_RunOnce(void);
/* USER CODE BEGIN PD */
#define DRV8703_EXAMPLE_CHANNEL DRV8703_BOARD_CH1
#define DRV8703_EXAMPLE_VREF_MV 2000U
#define DRV8703_EXAMPLE_DUTY 0.40f
/* USER CODE END PD */

/* USER CODE BEGIN Variables */
volatile uint8_t g_app_drv8703_selected_channel = DRV8703_EXAMPLE_CHANNEL;
volatile DRV8703_Status_t g_app_drv8703_init_result = DRV8703_ERROR_PARAM;
volatile DRV8703_Status_t g_app_drv8703_config_result = DRV8703_ERROR_PARAM;
volatile DRV8703_Status_t g_app_drv8703_start_result = DRV8703_ERROR_PARAM;
volatile DRV8703_Status_t g_app_drv8703_monitor_result = DRV8703_ERROR_PARAM;
volatile uint8_t g_app_drv8703_loop_counter = 0;
DRV8703_FaultInfo_t g_app_drv8703_fault_info = {0xFFU, 0xFFU};
uint8_t g_app_drv8703_reg_dump[DRV8703_REGISTER_COUNT] = {0};
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
  DRV8703_Handle_t *drv8703;

  osDelay(1000);

  drv8703 = DRV8703_BoardGet(DRV8703_EXAMPLE_CHANNEL);
  g_app_drv8703_init_result = DRV8703_BoardInitOne(DRV8703_EXAMPLE_CHANNEL);

  if ((drv8703 != 0) && (g_app_drv8703_init_result == DRV8703_OK))
  {
    g_app_drv8703_config_result =
        DRV8703_BoardApplyDefaultConfig(DRV8703_EXAMPLE_CHANNEL);

    if (g_app_drv8703_config_result == DRV8703_OK)
    {
      (void)DRV8703_SetVrefMv(drv8703, DRV8703_EXAMPLE_VREF_MV);
      (void)DRV8703_Wake(drv8703);
      g_app_drv8703_start_result =
          DRV8703_SetDuty(drv8703, DRV8703_EXAMPLE_DUTY);
    }
  }

  ADC1_Test_Init();
  ADC1_Test_CalibrateCurrentZero(64);

  for (;;)
  {
    ADC1_Test_RunOnce();
    (void)DRV8703_ReadFaultInfo(drv8703, &g_app_drv8703_fault_info);
    if (drv8703 != 0)
    {
      g_app_drv8703_monitor_result = DRV8703_CheckFaultPins(drv8703);
      (void)DRV8703_ReadFaultInfo(drv8703, &g_app_drv8703_fault_info);
      (void)DRV8703_DumpRegs(drv8703,
                             g_app_drv8703_reg_dump,
                             DRV8703_REGISTER_COUNT);

      if ((g_app_drv8703_monitor_result != DRV8703_OK) ||
          ((g_app_drv8703_fault_info.fault_status & DRV8703_FAULT_FAULT) != 0U))
      {
        (void)DRV8703_ReadFaultInfo(drv8703, &g_app_drv8703_fault_info);
        (void)DRV8703_SetDuty(drv8703, 0.0f);
        (void)DRV8703_Sleep(drv8703);
      }
    }

    /*
     * Keil Watch:
     * g_app_drv8703_*              DRV8703 driver status
     * g_app_drv8703_fault_info     FAULT and VDS/GDF status registers
     * g_app_drv8703_reg_dump[0..5] register snapshot
     * g_adc_v1_input_v ~ g_adc_v5_input_v
     * g_adc_i1_a       ~ g_adc_i5_a
     */
    g_app_drv8703_loop_counter++;
    osDelay(200);
  }
  for (;;)
  {
    osDelay(200);
  }

  /* USER CODE END StartControlask */
}

void StartHMITask(void *argument)
{
  /* USER CODE BEGIN StartHMITask */
  // (void)argument;
  // uint8_t led8_on = 0U;

  // TM1638_SetAllLEDs(&htm1638, 0U);

  // for (;;)
  // {
  //   led8_on ^= 1U;
  //   TM1638_SetLED(&htm1638, 8U, led8_on != 0U);
  //   osDelay(1000);
  // }

#if 1
  uint32_t now;
  static uint32_t last_tick = 0;

  for (;;)
  {
    now = osKernelGetTickCount();

    if (now - last_tick >= 20)
    {
      last_tick = now;
      TM1638_ProcessKeys(&htm1638);
      CheckKeyHoldEvents();
      TempPanel_Task(&g_panel, now);

      static uint32_t last_blink = 0;
      if (now - last_blink >= 500)
      {
        last_blink = now;
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      }
    }

    osDelay(5);
  }
#endif
  /* USER CODE END StartHMITask */
}

/* USER CODE BEGIN Application */
/* USER CODE END Application */
