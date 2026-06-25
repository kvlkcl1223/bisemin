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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bridge1_test.h"
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
uint8_t g_reg0 = 0;
uint8_t g_reg1 = 0;
uint8_t g_reg2 = 0;
uint8_t g_reg3 = 0;
uint8_t g_reg4 = 0;
uint8_t g_reg5 = 0;
uint8_t g_reg2_lock = 0;
uint8_t g_reg2_unlock = 0;
uint8_t g_reg3_after = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float Sys_Temperatures[4] = {0.0f};
volatile uint8_t Sys_Status[4] = {0};
volatile uint32_t Sys_TempUpdateCount[4] = {0};
volatile uint32_t Sys_TempUpdateTick[4] = {0};
volatile uint8_t g_uart_need_restart = 0U;
#define RX_BUFFER_SIZE 256
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_length = 0;
int8_t flag_temp_update = 0;
PID_TypeDef temp_pid;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Parse_Temperature_Buffer(uint8_t *pData, uint16_t Size);
void Parse_Temperature_Byte(uint8_t rx_byte);
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
      if (held >= 500)
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
void OnSwitchKeyPressed(void) { OnTM1638Key(KEY_SWITCH); }
uint8_t DRV8703_ReadReg1(uint8_t addr)
{
  uint8_t tx[2];
  uint8_t rx[2];
  uint16_t frame;

  /*
   * DRV8703 读命令：
   * bit15 = 1
   * bit14~11 = addr
   */
  frame = 0x8000U | ((uint16_t)(addr & 0x0F) << 11);

  tx[0] = (uint8_t)(frame >> 8);
  tx[1] = (uint8_t)(frame & 0xFF);

  rx[0] = 0;
  rx[1] = 0;

  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);

  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);

  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);

  /*
   * 返回低 8 bit
   */
  return rx[1];
}
void DRV8703_WriteReg1(uint8_t addr, uint8_t data)
{
  uint8_t tx[2];
  uint8_t rx[2];
  uint16_t frame;

  /*
   * DRV8703 写命令：
   * bit15 = 0
   * bit14~11 = addr
   * bit7~0 = data
   */
  frame = ((uint16_t)(addr & 0x0F) << 11) | data;

  tx[0] = (uint8_t)(frame >> 8);
  tx[1] = (uint8_t)(frame & 0xFF);

  rx[0] = 0;
  rx[1] = 0;

  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);

  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);

  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
}
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_DAC3_Init();
  MX_DAC4_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_OPAMP4_Init();
  MX_OPAMP5_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  // 开启 TIM3 的 PWM 输出，开启水泵的电机
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 8499);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 4000);

  // 启动带有空闲检测的 DMA 接收
  // 启动 USART2 带有空闲检测的 DMA 接收
  TemperatureUart_RestartReceive();
  // 直接通过 huart2 结构体内部的 hdmarx 指针来关闭过半中断
  HAL_Delay(20);
  HAL_GPIO_WritePin(NRST_OTHER_GPIO_Port, NRST_OTHER_Pin, GPIO_PIN_SET);

  /* 一键初始化: TM1638 + Panel + PID + 默认显示 */
  Panel_Init();

  /* 注册所有按键回调 */
  TM1638_RegisterKeyCallback(&htm1638, KEY_MODE, OnModeKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_START_STOP, OnStartStopKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_UP, OnUpKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_DOWN, OnDownKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_ENTER, OnEnterKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_SWITCH, OnSwitchKeyPressed);
  // HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
  // HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
  //                  (uint32_t)(1000.0f * 4095.0f / 3300.0f));
  // HAL_OPAMP_Start(&hopamp1);

  //  Bridge1_StartHoldTest(0.05f, 1000);

  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 150);

  //  HAL_GPIO_WritePin(SLEEP1_GPIO_Port, SLEEP1_Pin, GPIO_PIN_SET);
  //  HAL_Delay(20);

  //  /* 所有其它 SPI 片选都拉高 */
  //  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
  //  /* ADS1220_CS 也要拉高 */
  //  /* CS2/CS3/CS4/CS5 也要拉高 */

  //  /* 读默认寄存器 */
  //  g_reg0 = DRV8703_ReadReg1(0x00);
  //  g_reg1 = DRV8703_ReadReg1(0x01);
  //  g_reg2 = DRV8703_ReadReg1(0x02);
  //  g_reg3 = DRV8703_ReadReg1(0x03);
  //  g_reg4 = DRV8703_ReadReg1(0x04);
  //  g_reg5 = DRV8703_ReadReg1(0x05);

  //  /* 测试 Reg2 写入 */
  //  DRV8703_WriteReg1(0x02, 0x30);
  //  HAL_Delay(2);
  //  g_reg2_lock = DRV8703_ReadReg1(0x02);

  //  DRV8703_WriteReg1(0x02, 0x18);
  //  HAL_Delay(2);
  //  g_reg2_unlock = DRV8703_ReadReg1(0x02);

  //  /* 测试 Reg3 写入 */
  //  DRV8703_WriteReg1(0x03, 0xC7);
  //  HAL_Delay(2);
  //  g_reg3_after = DRV8703_ReadReg1(0x03);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
void TemperatureUart_RestartReceive(void)
{
  (void)HAL_UART_AbortReceive(&huart1);
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
  if (huart1.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    /* Parse temperature data */
    Parse_Temperature_Buffer(rx_buffer, Size);

    /*
     * Restart DMA idle-line reception directly in the ISR.
     * The idle line means the current frame is complete and DMA has stopped,
     * so we do NOT need HAL_UART_AbortReceive (which is blocking).
     * Just start a new DMA reception immediately.
     */
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
    if (huart1.hdmarx != NULL)
    {
      __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
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
            Sys_TempUpdateCount[ch_id - 1]++;
            Sys_TempUpdateTick[ch_id - 1] = HAL_GetTick();

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
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
