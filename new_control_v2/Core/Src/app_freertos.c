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
/* Definitions for SysStateMutex */
osMutexId_t SysStateMutexHandle;
const osMutexAttr_t SysStateMutex_attributes = {
  .name = "SysStateMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static HAL_StatusTypeDef App_StartSynchronizedPwmTimebase(void);
static void AppDebug_UartSend(const char *str);
static void AppDebug_LogNormal(uint32_t now_ms);
/* USER CODE END FunctionPrototypes */

void StartControlTask(void *argument);
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

  AppDebug_UartSend("SYS,CONTROL_INIT_BEGIN\r\n");

  g_app_tim_sync_start_result = App_StartSynchronizedPwmTimebase();
  {
    char buf[48];
    (void)snprintf(buf, sizeof(buf),
                   "SYS,TIM_SYNC:%u\r\n",
                   (unsigned int)g_app_tim_sync_start_result);
    AppDebug_UartSend(buf);
  }

  g_app_adc_measure_start_result = AdcMeasure_Start();
  {
    char buf[48];
    (void)snprintf(buf, sizeof(buf),
                   "SYS,ADC_START:%u\r\n",
                   (unsigned int)g_app_adc_measure_start_result);
    AppDebug_UartSend(buf);
  }

  if (g_app_adc_measure_start_result == ADC_MEASURE_OK)
  {
    AdcMeasure_CalibrateCurrentZero(APP_ADC_MEASURE_ZERO_SAMPLES);
    AppDebug_UartSend("SYS,ADC_ZERO_DONE\r\n");
  }
  g_app_control_start_result = AppControl_Init();
  {
    char buf[48];
    (void)snprintf(buf, sizeof(buf),
                   "SYS,APP_CONTROL_INIT:%u\r\n",
                   (unsigned int)g_app_control_start_result);
    AppDebug_UartSend(buf);
  }

  /* TODO: 临时 Flash 读写测试，验证通过后删除 */
  CalibMode_FlashTest(0);
  CalibMode_FlashTest(1);

  AppDebug_UartSend("CALIB,AUTO_START,CELL:0\r\n");
  CalibMode_Start(0);

  /* Infinite loop */
  for (;;)
  {
    if (g_uart_need_restart != 0U)
    {
      TemperatureUart_RestartReceive();
    }

    AdcMeasure_Process();
    AppControl_Task(osKernelGetTickCount());
    AppDebug_LogNormal(osKernelGetTickCount());

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
        if (g_calib_state == CALIB_INIT)
        {
          /* INIT: LED1 亮，数码管清空 */
          TM1638_SetAllLEDs(&htm1638, 0x01);
          TM1638_ClearDisplay(&htm1638);
        }
        else if (g_calib_state == CALIB_RUN || g_calib_state == CALIB_IDLE)
        {
          /* RUN/IDLE 过渡态: LED 全灭，数码管清空 */
          TM1638_SetAllLEDs(&htm1638, 0x00);
          TM1638_ClearDisplay(&htm1638);
        }
        else if (g_calib_state == CALIB_WAIT_STABLE)
        {
          /* WAIT_STABLE: 三个值每 2s 轮换，LED1/2/3 分别指示 */
          uint8_t disp_mode = (uint8_t)((now / 2000U) % 3U);
          TM1638_ClearDisplay(&htm1638);

          if (disp_mode == 0U)
          {
            /* 占空比 → LED1 */
            float duty = CalibMode_StepDuty((uint8_t)g_calib_step_idx);
            TM1638_SetAllLEDs(&htm1638, 0x01);
            TM1638_ShowFloat(&htm1638, duty, 2);
          }
          else if (disp_mode == 1U)
          {
            /* CH0 温度 → LED2 */
            TM1638_SetAllLEDs(&htm1638, 0x02);
            TM1638_ShowFloat(&htm1638, g_calib_temp_ch0, 1);
          }
          else
          {
            /* CH1 温度 → LED3 */
            TM1638_SetAllLEDs(&htm1638, 0x04);
            TM1638_ShowFloat(&htm1638, g_calib_temp_ch1, 1);
          }
        }
        else if (g_calib_state == CALIB_DONE)
        {
          /* DONE: 9 LED 全亮，数码管清空 */
          TM1638_SetAllLEDs(&htm1638, 0x1FF);
          TM1638_ClearDisplay(&htm1638);
        }
        else if (g_calib_state == CALIB_FAULT)
        {
          /* FAULT: 交替 LED，数码管显示 Err */
          TM1638_SetAllLEDs(&htm1638, 0x155);
          TM1638_ClearDisplay(&htm1638);
          TM1638_ShowString(&htm1638, "Err");
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
static void AppDebug_UartSend(const char *str)
{
  uint16_t len = 0U;
  const char *p = str;

  if (str == NULL)
    return;

  while (*p != '\0')
  {
    len++;
    p++;
  }

  if (len > 0U)
  {
    (void)HAL_UART_Transmit(&huart2, (const uint8_t *)str, len, 20U);
  }
}

static void AppDebug_LogNormal(uint32_t now_ms)
{
  static uint32_t s_last_log_ms = 0U;
  static uint8_t s_normal_start_sent = 0U;
  char buf[192];
  int len;

  if (g_calib_mode_active != 0U)
  {
    s_normal_start_sent = 0U;
    return;
  }

  if (s_normal_start_sent == 0U)
  {
    s_normal_start_sent = 1U;
    AppDebug_UartSend("APP,NORMAL_START\r\n");
  }

  if ((now_ms - s_last_log_ms) < 1000U)
    return;
  s_last_log_ms = now_ms;

  len = snprintf(buf, sizeof(buf),
                 "APP,RUN,C0:%u,T0:%.2f,TAR0:%.2f,DUTY0:%.3f,"
                 "C1:%u,T1:%.2f,TAR1:%.2f,DUTY1:%.3f,"
                 "TCNT:%lu,%lu,%lu,%lu\r\n",
                 (unsigned int)g_app_control_cell_running[0],
                 (double)g_app_control_cell_temp[0],
                 (double)g_app_control_cell_target[0],
                 (double)g_app_control_cell_duty[0],
                 (unsigned int)g_app_control_cell_running[1],
                 (double)g_app_control_cell_temp[1],
                 (double)g_app_control_cell_target[1],
                 (double)g_app_control_cell_duty[1],
                 (unsigned long)g_app_control_temp_update_count[0],
                 (unsigned long)g_app_control_temp_update_count[1],
                 (unsigned long)g_app_control_temp_update_count[2],
                 (unsigned long)g_app_control_temp_update_count[3]);

  if (len > 0 && len < (int)sizeof(buf))
  {
    AppDebug_UartSend(buf);
  }
}
/* USER CODE END Application */

