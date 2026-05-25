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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tm1638_board.h"
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
TM1638_HandleTypeDef htm1638;

// 定义温度存储数组 (索引 0~3 对应 CH1~CH4)
float Sys_Temperatures[4] = {0.0f};

// 定义一个状态数组，记录探头是否断开 (0:正常, 1:断开)
uint8_t Sys_Status[4] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Parse_Temperature_Buffer(uint8_t *pData, uint16_t Size);
void Parse_Temperature_Byte(uint8_t rx_byte);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 1. 实现底层的硬件接口函数 */
void HW_SetSTB(bool state)
{
  HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void HW_SetCLK(bool state)
{
  HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void HW_SetDIO(bool state)
{
  HAL_GPIO_WritePin(DIO_GPIO_Port, DIO_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool HW_GetDIO(void)
{
  return HAL_GPIO_ReadPin(DIO_GPIO_Port, DIO_Pin) == GPIO_PIN_SET;
}

void HW_DelayUs(uint32_t us)
{
  // STM32 简单的微秒延时（可根据主频调整，或者使用定时器，由于只要1-2us，用空循环即可）
  uint32_t delay = us * (SystemCoreClock / 1000000 / 4);
  while (delay--)
  {
    __NOP();
  }
}
uint8_t count = 0;
/* 2. 定义按键被按下时的业务逻辑 (回调函数) */
void OnModeKeyPressed(void)
{
  count++;
  if (count > 9)
    count = 0;
  TM1638_ShowDigit(&htm1638, 3, count, false); // 第4位数码管显示数字
  // MODE 键按下的逻辑
  TM1638_SetLED(&htm1638, 1, true); // 点亮 LED1 (NORMAL)
}

void OnStartStopKeyPressed(void)
{
  // START/STOP 键按下的逻辑
  TM1638_SetLED(&htm1638, 5, true); // 点亮 LED5 (STOP)
}

void OnUpKeyPressed(void)
{
  count++;
  if (count > 9)
    count = 0;
  TM1638_ShowDigit(&htm1638, 3, count, false); // 第4位数码管显示数字
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

  // 启动带有空闲检测的 DMA 接收
  // 1. 启动 USART2 带有空闲检测的 DMA 接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE);

  // 2. 修正报错：直接通过 huart2 结构体内部的 hdmarx 指针来关闭过半中断
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);

  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);

  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000 - 1);

  HAL_Delay(1000);
  // 装配底层接口并初始化
  TM1638_HW_t hw = {HW_SetSTB, HW_SetCLK, HW_SetDIO, HW_GetDIO, HW_DelayUs};
  TM1638_Init(&htm1638, &hw);
  // 设置亮度 (0-7), 开启显示
  TM1638_SetBrightness(&htm1638, 5, true);
  // 注册按键回调函数
  TM1638_RegisterKeyCallback(&htm1638, KEY_MODE, OnModeKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_START_STOP, OnStartStopKeyPressed);
  TM1638_RegisterKeyCallback(&htm1638, KEY_UP, OnUpKeyPressed);
  // 开机显示一些默认状态
  TM1638_ShowFloat(&htm1638, -5.2f, 1);
  TM1638_SetLED(&htm1638, 4, true);
  TM1638_SetLED(&htm1638, 5, true);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000 - 1);

  HAL_Delay(100);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 100);
  HAL_Delay(100);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 200);
  HAL_Delay(100);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 300);
  HAL_Delay(100);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 400);
  HAL_Delay(100);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 600);
  HAL_Delay(100);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 800);
  //		HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 900);
  //			HAL_Delay(1000);
  //	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1000);

  //
  //
  //		HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
  //	HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);
  //	HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 300);
  //	HAL_Delay(100);
  //	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 350);
  //
  //
  ////		HAL_Delay(100);
  ////	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 600);
  ////		HAL_Delay(100);
  ////	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 700);
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
  HAL_GPIO_WritePin(NRST_OTHER_GPIO_Port, NRST_OTHER_Pin, GPIO_PIN_SET);
  uint32_t last_scan_time = 0;
  float temp = 0.0f;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // temp += 0.1f;
    // TM1638_ShowFloat(&htm1638, temp, 1);
    // if (HAL_GetTick() - last_scan_time >= 20)
    // {
    //   last_scan_time = HAL_GetTick();

    //   // 这个函数内部会自动读取按键，并在有按键按下时，触发我们上面注册的 OnxxxKeyPressed()
    //   TM1638_ProcessKeys(&htm1638);
    // }
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(1000);
    TM1638_ShowFloat(&htm1638, Sys_Temperatures[0], 1);
    TM1638_SetLED(&htm1638, 4, true);
    TM1638_SetLED(&htm1638, 5, false);
    HAL_Delay(1000);
    TM1638_ShowFloat(&htm1638, Sys_Temperatures[1], 1);
    TM1638_SetLED(&htm1638, 4, false);
    TM1638_SetLED(&htm1638, 5, true);
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

            // 【可选】在这里可以通过 USART1 打印出来查看解析结果
            // printf("CH%d OK: %.4f\r\n", ch_id, temp_val);
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
