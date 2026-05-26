/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tm1638_board.h"
#include "pid_controller.h"
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

/* USER CODE BEGIN PV */
#define RX_BUFFER_SIZE 256
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_length = 0;

PID_TypeDef temp_pid;

float Sys_Temperatures[4] = {0.0f};
uint8_t Sys_Status[4] = {0};

int16_t count_tim4 = 0;
int8_t flag_temp_update = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void Parse_Temperature_Buffer(uint8_t *pData, uint16_t Size);
void Parse_Temperature_Byte(uint8_t rx_byte);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ── 长按检测 ── */
static uint32_t key_hold_start[KEY_MAX_NUM] = {0};
static uint32_t key_last_repeat[KEY_MAX_NUM] = {0};
static bool key_long_sent[KEY_MAX_NUM] = {false};

/* 按键回调: 仅在按下瞬间触发 SHORT 事件, 同时记录 hold 起始时刻 */
static void OnTM1638Key(TM1638_Key_t key)
{
  PanelKey_t pk = PanelKey_FromTM1638(key);
  if (pk == PANEL_KEY_NONE)
    return;

  uint32_t now = osKernelGetTickCount();
  key_hold_start[key] = now;
  key_last_repeat[key] = 0;
  key_long_sent[key] = false;
  TempPanel_KeyEvent(&g_panel, pk, PANEL_KEY_EVT_SHORT, now);
}

/*
 * 长按/连发检测: 需在主循环或 HMITask 中每 20ms 调用,
 * 紧跟在 TM1638_ProcessKeys() 之后。
 */
void CheckKeyHoldEvents(void)
{
  uint32_t now = osKernelGetTickCount();
  for (int i = 0; i < KEY_MAX_NUM; i++)
  {
    PanelKey_t pk = PanelKey_FromTM1638((TM1638_Key_t)i);
    if (pk == PANEL_KEY_NONE)
      continue;

    if (htm1638.key_states[i])
    {
      if (key_hold_start[i] == 0)
        continue;

      uint32_t held = now - key_hold_start[i];

      /* 按住 > 2s 触发一次 LONG */
      if (held >= 2000)
      {
        if (!key_long_sent[i])
        {
          TempPanel_KeyEvent(&g_panel, pk, PANEL_KEY_EVT_LONG, now);
          key_long_sent[i] = true;
        }
      }
      /* 按住 > 500ms 开始连发, 每 150ms 一次 REPEAT */
      else if (held >= 500)
      {
        if (key_last_repeat[i] == 0 || now - key_last_repeat[i] >= 150)
        {
          TempPanel_KeyEvent(&g_panel, pk, PANEL_KEY_EVT_REPEAT, now);
          key_last_repeat[i] = now;
        }
      }
    }
    else
    {
      key_hold_start[i] = 0;
      key_last_repeat[i] = 0;
      key_long_sent[i] = false;
    }
  }
}

void OnModeKeyPressed(void) { OnTM1638Key(KEY_MODE); }
void OnStartStopKeyPressed(void) { OnTM1638Key(KEY_START_STOP); }
void OnUpKeyPressed(void) { OnTM1638Key(KEY_UP); }
void OnDownKeyPressed(void) { OnTM1638Key(KEY_DOWN); }
void OnEnterKeyPressed(void) { OnTM1638Key(KEY_ENTER); }
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  //  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  //  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  //  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_4);

  //  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  //  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  //  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  //  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  //  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  //  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
  //  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  //  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_4);

  //  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  //  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);

  //  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  //  HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);

  //  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  //  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

  //  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  //  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  //  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
  //  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);

  //  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
  //  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);

  //  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
  //  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000 - 1);

  // 启动带有空闲检测的 DMA 接收
  // 1. 启动 USART2 带有空闲检测的 DMA 接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE);

  // 2. 修正报错：直接通过 huart2 结构体内部的 hdmarx 指针来关闭过半中断
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  HAL_Delay(1000);

  /* 一键初始化: TM1638 + Panel + PID + 默认显示 */
  Panel_Init();

  /* 注册所有按键回调 */
  TM1638_RegisterKeyCallback(&htm1638, KEY_MODE, OnModeKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_START_STOP, OnStartStopKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_UP, OnUpKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_DOWN, OnDownKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_ENTER, OnEnterKeyPressed);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000 - 1);

  HAL_Delay(100);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 200);
  HAL_Delay(100);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 400);
  HAL_Delay(100);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 700);

  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 100);
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 200);
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 300);
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 400);
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 600);
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 800);
  // //		HAL_Delay(100);
  // //	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 900);
  // //			HAL_Delay(1000);
  // //	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1000);

  // //
  // //
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 300);
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 350);
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 600);
  // HAL_Delay(100);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 700);
  ////		HAL_Delay(100);
  ////	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 900);
  //
  //
  //			HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 100);
  //	HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 200);
  //	HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 300);
  //	HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 400);
  //		HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 600);
  //		HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 700);
  //		HAL_Delay(100);
  ////	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 900);
  ////			HAL_Delay(100);
  ////	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
  //

  // // 1. 初始化 PID: Kp=2.0, Ki=5.0, Kd=0.1, 采样周期 dt=0.01秒 (即 100Hz)
  // PID_Init(&temp_pid, 30.0f, 1.0f, 0.0f, 0.1f);

  // // 2. 设置限幅: 输出限制在 -100 到 100 (占空比), 积分限制在 -40 到 40
  // PID_SetLimits(&temp_pid, 700, 1699, 700, 1600);

  // // 3. 设置目标值：让电机达到 1500 RPM
  // PID_SetTarget(&temp_pid, -10.0f);

  //
  HAL_GPIO_WritePin(NRST_OTHER_GPIO_Port, NRST_OTHER_Pin, GPIO_PIN_SET);
  uint32_t last_scan_time = 0;
  float temp = 0.0f;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop (fallback: 如果 FreeRTOS 未启动，在此运行测试程序) */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    static uint32_t last_scan = 0;

    /* 每 20ms 扫描一次按键并驱动面板 */
    if (now - last_scan >= 20)
    {
      last_scan = now;
      TM1638_ProcessKeys(&htm1638);
      TempPanel_Task(&g_panel, now);
    }

    /* 将收到的温度数据注入面板 (float 直接传入) */
    if (flag_temp_update)
    {
      TempPanel_UpdateMeasuredTemp(&g_panel, 1, Sys_Temperatures[1], now);
      flag_temp_update = 0;
    }

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(10);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  // 判断是否是 USART2 触发的接收完成/空闲中断
  if (huart->Instance == USART2)
  {
    // 2. 将这次 DMA 收到的数据块扔给解析器提取温度
    Parse_Temperature_Buffer(rx_buffer, Size);

    // 3. 必须重新开启 USART2 的 DMA 接收，否则只能收一次
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE);

    // 4. 再次关闭过半中断，防止接收一半时触发不必要的中断打断 CPU
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
}

/**
 * @brief  字节级协议解析状态机 (自动处理粘包/断包)
 * @param  rx_byte 接收到的单字节
 */
void Parse_Temperature_Byte(uint8_t rx_byte)
{
  static uint8_t state = 0;
  static uint8_t frame_buf[9];
  static uint8_t idx = 0;

  switch (state)
  {
  case 0: // 寻找帧头 1
    if (rx_byte == 0xAA)
    {
      frame_buf[0] = rx_byte;
      state = 1;
    }
    break;

  case 1: // 寻找帧头 2
    if (rx_byte == 0x55)
    {
      frame_buf[1] = rx_byte;
      idx = 2;
      state = 2;
    }
    else if (rx_byte == 0xAA)
    {
      state = 1; // 容错：连续收到两个 AA
    }
    else
    {
      state = 0; // 帧头错误，状态机复位
    }
    break;

  case 2: // 接收数据体与校验和
    frame_buf[idx++] = rx_byte;

    if (idx >= 9)
    { // 收集满一帧 9 字节
      // 1. 计算校验和
      uint8_t sum = 0;
      for (int i = 2; i <= 7; i++)
      {
        sum += frame_buf[i];
      }

      // 2. 验证校验和
      if (sum == frame_buf[8])
      {
        uint8_t ch_id = frame_buf[2];  // 通道号 1~4
        uint8_t status = frame_buf[3]; // 状态位

        // 3. 提取并保存数据
        if (ch_id >= 1 && ch_id <= 4)
        {
          Sys_Status[ch_id - 1] = status; // 保存状态

          if (status == 0)
          {
            // 状态正常时，提取浮点温度数据
            float temp_val;
            uint8_t *pTemp = (uint8_t *)&temp_val;
            pTemp[0] = frame_buf[4];
            pTemp[1] = frame_buf[5];
            pTemp[2] = frame_buf[6];
            pTemp[3] = frame_buf[7];

            // 存入全局温度数组
            Sys_Temperatures[ch_id - 1] = temp_val;

            // printf("CH%d OK: %.4f\r\n", ch_id, temp_val);
            if (ch_id == 2) // 如果是 CH2 的温度更新了，设置标志让主循环计算 PID 输出
            {
              flag_temp_update = 1;
            }
          }
          else
          {
            // 如果状态为 1 (断开/超时)，为了安全，可以把该通道温度设为一个异常值 (例如 -999.0)
            // Sys_Temperatures[ch_id - 1] = -999.0f;
          }
        }
      }
      // 解析完毕，状态机归零，准备迎接下一帧
      state = 0;
    }
    break;
  }
}

/**
 * @brief  数据块解析函数 (供空闲中断调用)
 * @param  pData 接收到的数据首地址
 * @param  Size  接收到的数据长度
 */
void Parse_Temperature_Buffer(uint8_t *pData, uint16_t Size)
{
  for (uint16_t i = 0; i < Size; i++)
  {
    Parse_Temperature_Byte(pData[i]); // 将数据块逐字节喂给状态机
  }
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
