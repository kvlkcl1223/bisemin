/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
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
#include "adc_measure.h"
#include "app_control.h"
#include "calib_mode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DRV8703_EXAMPLE_CHANNEL DRV8703_BOARD_CH1
#define DRV8703_EXAMPLE_VREF_MV 2000U
#define DRV8703_EXAMPLE_DUTY 0.40f
#define APP_ADC_MEASURE_ZERO_SAMPLES 64U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint8_t g_app_drv8703_selected_channel = DRV8703_EXAMPLE_CHANNEL;
volatile DRV8703_Status_t g_app_drv8703_init_result = DRV8703_ERROR_PARAM;
volatile DRV8703_Status_t g_app_drv8703_config_result = DRV8703_ERROR_PARAM;
volatile DRV8703_Status_t g_app_drv8703_start_result = DRV8703_ERROR_PARAM;
volatile DRV8703_Status_t g_app_drv8703_monitor_result = DRV8703_ERROR_PARAM;
volatile DRV8703_Status_t g_app_drv8703_read_fault_result = DRV8703_ERROR_PARAM;
volatile DRV8703_Status_t g_app_drv8703_dump_result = DRV8703_ERROR_PARAM;
volatile uint8_t g_app_drv8703_loop_counter = 0;
DRV8703_FaultInfo_t g_app_drv8703_fault_info = {0xFFU, 0xFFU};
uint8_t g_app_drv8703_reg_dump[DRV8703_REGISTER_COUNT] = {0};
volatile HAL_StatusTypeDef g_app_tim_sync_start_result = HAL_OK;
volatile uint32_t g_app_tim1_trgo2_compare = 0U;
volatile AdcMeasure_Status_t g_app_adc_measure_start_result = ADC_MEASURE_ERROR_ADC1;
volatile AppControl_Status_t g_app_control_start_result = APP_CONTROL_ERROR_QUEUE;
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
static HAL_StatusTypeDef App_StartSynchronizedPwmTimebase(void);
/* USER CODE END FunctionPrototypes */

void StartControlTask(void *argument);
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
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of HMITask */
  HMITaskHandle = osThreadNew(StartHMITask, NULL, &HMITask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
 * @brief  Function implementing the ControlTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  (void)argument;

  osDelay(1000);

  g_app_tim_sync_start_result = App_StartSynchronizedPwmTimebase();
  g_app_adc_measure_start_result = AdcMeasure_Start();
  if (g_app_adc_measure_start_result == ADC_MEASURE_OK)
  {
    AdcMeasure_CalibrateCurrentZero(APP_ADC_MEASURE_ZERO_SAMPLES);
  }
  g_app_control_start_result = AppControl_Init();
  CalibMode_Start(0);

  /* Infinite loop */
  for (;;)
  {
    AdcMeasure_Process();
    AppControl_Task(osKernelGetTickCount());

    /*
     * Keil Watch:
     * g_app_control_*              control task status
     * g_adc_v1_input_v ~ g_adc_v5_input_v
     * g_adc_i1_a       ~ g_adc_i5_a
     * g_adc_il1_a      ~ g_adc_il5_a
     * g_adc_measure_adc1_update_count / g_adc_measure_adc2_update_count
     */
    g_app_drv8703_loop_counter++;
    osDelay(20);
  }
  /* USER CODE END StartControlTask */
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

      if (g_calib_mode_active != 0U)
      {
        uint16_t led_mask = 0U;
        if (g_calib_state == CALIB_IDLE || g_calib_state == CALIB_RUN)
          led_mask = 0x00;
        else if (g_calib_state == CALIB_INIT)
          led_mask = 0x01;
        else if (g_calib_state == CALIB_WAIT_STABLE)
          led_mask = 0x03;
        else if (g_calib_state == CALIB_DONE)
          led_mask = 0x1FF;
        else if (g_calib_state == CALIB_FAULT)
          led_mask = 0x155;
        TM1638_SetAllLEDs(&htm1638, led_mask);

        TM1638_ClearDisplay(&htm1638);
        if (g_calib_state == CALIB_DONE)
        {
        }
        else if (g_calib_state == CALIB_FAULT)
        {
          TM1638_ShowString(&htm1638, "Err");
        }
        else
        {
          TM1638_ShowChar(&htm1638, 0U, 'C', false);
          TM1638_ShowDigit(&htm1638, 1U, g_calib_cell, false);
          TM1638_ShowDigit(&htm1638, 3U, g_calib_step_idx / 10U, false);
          TM1638_ShowDigit(&htm1638, 4U, g_calib_step_idx % 10U, false);
          if (g_calib_show_duty != 0U)
          {
            float duty = CalibMode_StepDuty(g_calib_step_idx);
            TM1638_ShowChar(&htm1638, 5U, 'd', false);
            TM1638_ShowFloat(&htm1638, duty, 2);
          }
          else
          {
            float temp;
            if (g_calib_cell == 0U)
              temp = g_calib_temp_ch0;
            else
              temp = g_calib_temp_ch1;
            TM1638_ShowChar(&htm1638, 5U, 't', false);
            TM1638_ShowFloat(&htm1638, temp, 1);
          }
        }
      }
      else
      {
        TM1638_ProcessKeys(&htm1638);
        CheckKeyHoldEvents();
        AppControl_UpdatePanel(&g_panel, now);
        TempPanel_Task(&g_panel, now);
      }

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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static HAL_StatusTypeDef App_TIM_BaseStartIfStopped(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance->CR1 & TIM_CR1_CEN) != 0U)
  {
    return HAL_OK;
  }

  return HAL_TIM_Base_Start(htim);
}

static HAL_StatusTypeDef App_StartSynchronizedPwmTimebase(void)
{
  HAL_StatusTypeDef ret;
  uint32_t arr;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);

  /*
   * TIM1 TRGO2 is OC5REF. CH5 has no pin output, so it is only used as
   * the ADC2 sampling trigger. PWM2 + CCR5 at mid-period gives a rising
   * edge near the middle of the PWM cycle.
   */
  arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  g_app_tim1_trgo2_compare = (arr + 1U) / 2U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, g_app_tim1_trgo2_compare);

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);

  ret = App_TIM_BaseStartIfStopped(&htim2);
  if (ret != HAL_OK)
  {
    return ret;
  }

  return App_TIM_BaseStartIfStopped(&htim1);
}
/* USER CODE END Application */
