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
#include <stdio.h>
#include <math.h>
#include <string.h> // ���� strlen
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ---------------- 1. 硬件引脚映射 ----------------
const uint16_t CS_PINS[4] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
const uint16_t DRDY_PINS[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10};

#define CMD_RESET 0x06
#define CMD_START_SYNC 0x08
#define CMD_RDATA 0x10
#define FILTER_N 10

// ---------------- 2. 底层 SPI 与安全互斥片选 ----------------
static uint16_t last_cs_pin = 0; // 记住上次选中的CS，O(1)替代O(n)遍历

void Safe_CS_Select(uint16_t target_cs_pin)
{
    // 仅关断上一次选中的通道
    if (last_cs_pin != 0 && last_cs_pin != target_cs_pin)
    {
        HAL_GPIO_WritePin(GPIOB, last_cs_pin, GPIO_PIN_SET);
    }
    // 拉低(打开)当前需要的通道
    HAL_GPIO_WritePin(GPIOB, target_cs_pin, GPIO_PIN_RESET);
    last_cs_pin = target_cs_pin;
    // 满足手册 t_d(CSSC) ≥ 50ns (CS下降沿到第一个SCLK上升沿)
    __NOP();
    __NOP();
}

void Safe_CS_Deselect(uint16_t target_cs_pin)
{
    HAL_GPIO_WritePin(GPIOB, target_cs_pin, GPIO_PIN_SET);
}

uint8_t ADS1220_SPI_Transfer(uint8_t data)
{
    uint8_t rxData = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rxData, 1, 100);
    return rxData;
}

void ADS1220_WriteReg(uint16_t cs_pin, uint8_t regAddr, uint8_t data)
{
    Safe_CS_Select(cs_pin);
    ADS1220_SPI_Transfer(0x40 | (regAddr << 2));
    ADS1220_SPI_Transfer(data);
    Safe_CS_Deselect(cs_pin);
}

// ---------------- 3. 初始化与读取逻辑 ----------------

// ---------------- 4. 算法部分 ----------------
// IEC 60751 Callendar-Van Dusen 公式 (PT1000), Newton-Raphson 迭代求解
float PT1000_ResistanceToTemperature(float resistance)
{
    const float R0 = 1000.0f;
    const float A = 3.9083e-3f;
    const float B = -5.775e-7f;
    const float C = -4.183e-12f; // 仅负温区使用

    // 初始估计：线性近似
    float t = (resistance - R0) / (R0 * A);
    if (t > 850.0f)
        t = 850.0f;
    else if (t < -200.0f)
        t = -200.0f;

    // Newton-Raphson 迭代 (最多5次, 收敛阈值 0.0001°C)
    for (int iter = 0; iter < 5; iter++)
    {
        float rt, deriv;
        if (t >= 0.0f)
        {
            rt = R0 * (1.0f + A * t + B * t * t);
            deriv = R0 * (A + 2.0f * B * t);
        }
        else
        {
            float t2 = t * t;
            float t3 = t2 * t;
            rt = R0 * (1.0f + A * t + B * t2 + C * (t - 100.0f) * t3);
            deriv = R0 * (A + 2.0f * B * t + C * (4.0f * t3 - 300.0f * t2));
        }
        float delta = (resistance - rt) / deriv;
        t += delta;
        if (fabsf(delta) < 0.0001f)
            break;
    }
    return t;
}

float Moving_Average_Filter(uint8_t ch_idx, float new_temp)
{
    static float temp_buffer[4][FILTER_N] = {0}; // 4路独立的滤波池
    static uint8_t index[4] = {0};
    static uint8_t is_full[4] = {0};
    float sum = 0.0f;

    temp_buffer[ch_idx][index[ch_idx]] = new_temp;
    index[ch_idx]++;
    if (index[ch_idx] >= FILTER_N)
    {
        index[ch_idx] = 0;
        is_full[ch_idx] = 1;
    }

    if (!is_full[ch_idx])
        return new_temp;

    for (uint8_t i = 0; i < FILTER_N; i++)
    {
        sum += temp_buffer[ch_idx][i];
    }
    return sum / FILTER_N;
}
/**
 * @brief  初始化特定通道的 ADS1220 (完美适配 PT1000)
 */
void ADS1220_Init(uint16_t cs_pin)
{
    // 软件复位, 手册§8.5.3.1: RESET后需≥50μs+32×t_CLK≈58μs
    Safe_CS_Select(cs_pin);
    ADS1220_SPI_Transfer(CMD_RESET);
    Safe_CS_Deselect(cs_pin);
    HAL_Delay(1);

    // Reg0=0x32: MUX=0011(AIN_P=AIN1,AIN_N=AIN2), GAIN=001(2x), PGA启用
    ADS1220_WriteReg(cs_pin, 0x00, 0x32);
    // Reg1=0x04: DR=000(20SPS), MODE=00(正常), CM=1(连续转换), TS=0, BCS=0
    ADS1220_WriteReg(cs_pin, 0x01, 0x04);
    // Reg2=0x54: VREF=01(外部REFP0/REFN0), 50/60=01(同时抑制), IDAC=100b(250μA)
    ADS1220_WriteReg(cs_pin, 0x02, 0x54);
    // Reg3=0x20: I1MUX=001(IDAC1→AIN0), I2MUX=000(IDAC2禁用), DRDYM=0
    ADS1220_WriteReg(cs_pin, 0x03, 0x20);
}

/**
 * @brief  读取 24位 ADC 并解算阻值 (防死锁安全版)
 */
float ADS1220_Read_PT1000_Resistance(uint16_t cs_pin, uint16_t drdy_pin)
{
    uint32_t rawData = 0;
    int32_t signedData = 0;

    // 1. 发送同步命令触发这一次单次转换
    Safe_CS_Select(cs_pin);
    ADS1220_SPI_Transfer(CMD_START_SYNC);
    Safe_CS_Deselect(cs_pin);

    // 【致命修复3】必须等 2 毫秒！让 ADS1220 有时间把 DRDY 拉高，防止 STM32 读到上一轮的低电平！
    HAL_Delay(2);

    // 2. 死等 DRDY 引脚被拉低 (转换完成)
    uint32_t start_time = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOB, drdy_pin) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start_time) > 80)
            return -1.0; // 20SPS转换约50ms, 80ms足矣
    }

    // 3. 读取 24 位数据
    Safe_CS_Select(cs_pin);
    ADS1220_SPI_Transfer(CMD_RDATA);
    rawData |= ADS1220_SPI_Transfer(0xFF) << 16;
    rawData |= ADS1220_SPI_Transfer(0xFF) << 8;
    rawData |= ADS1220_SPI_Transfer(0xFF);
    Safe_CS_Deselect(cs_pin);

    // 4. 补码换算
    if (rawData & 0x800000)
    {
        signedData = (int32_t)(rawData | 0xFF000000);
    }
    else
    {
        signedData = (int32_t)rawData;
    }

    // 换算公式: 基准电阻 4k, 增益 2 倍
    return ((float)signedData * 4000.0f) / (2.0f * 8388608.0f);
}

/**
 * @brief  发送命令：并发触发转换专用
 */
void ADS1220_Start_Conversion(uint16_t cs_pin)
{
    Safe_CS_Select(cs_pin);
    ADS1220_SPI_Transfer(CMD_START_SYNC);
    Safe_CS_Deselect(cs_pin);
}

/**
 * @brief  纯净读取函数：不包含等待，直接去 SPI 总线拿 24 位数据
 */
// 手册§8.5.4: DRDY=0时可直接读24bit数据, 无需RDATA命令
float ADS1220_Fetch_PT1000_Resistance(uint16_t cs_pin)
{
    uint32_t rawData = 0;
    int32_t signedData = 0;

    Safe_CS_Select(cs_pin);
    rawData |= ADS1220_SPI_Transfer(0xFF) << 16;
    rawData |= ADS1220_SPI_Transfer(0xFF) << 8;
    rawData |= ADS1220_SPI_Transfer(0xFF);
    Safe_CS_Deselect(cs_pin);

    // 补码换算
    if (rawData & 0x800000)
    {
        signedData = (int32_t)(rawData | 0xFF000000);
    }
    else
    {
        signedData = (int32_t)rawData;
    }

    return ((float)signedData * 4000.0f) / (2.0f * 8388608.0f);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    char uart_buf[100];
    sprintf(uart_buf, "\r\n==================================\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);

    // 1. 初始化前，强行拉高所有 CS 引脚
    for (int i = 0; i < 4; i++)
    {
        HAL_GPIO_WritePin(GPIOB, CS_PINS[i], GPIO_PIN_SET);
    }
    HAL_Delay(100);

    // 2. 依次初始化 4 个通道的 ADS1220
    for (int i = 0; i < 4; i++)
    {
        ADS1220_Init(CS_PINS[i]);
        HAL_Delay(10);
    }

    sprintf(uart_buf, "4-CH PT1000 Initialization OK!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);

    // 3. 发送一次START/SYNC启动4路连续转换 (§8.4.2.2: CM=1时发一次即可)
    for (int i = 0; i < 4; i++)
    {
        ADS1220_Start_Conversion(CS_PINS[i]);
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        static char uart1_tx_buf[4][100];

        // ==========================================================
        // 连续转换模式: 只需等待4路DRDY被拉低, 每通道独立超时80ms
        // ==========================================================
        uint32_t global_start = HAL_GetTick();
        uint32_t ch_timeout[4] = {0}; // 各通道独立起始时间
        uint8_t ready_mask = 0x00;

        while (ready_mask != 0x0F)
        {
            uint32_t now = HAL_GetTick();
            for (int i = 0; i < 4; i++)
            {
                if ((ready_mask & (1 << i)) != 0)
                    continue; // 已就绪, 跳过
                if (HAL_GPIO_ReadPin(GPIOB, DRDY_PINS[i]) == GPIO_PIN_RESET)
                {
                    ready_mask |= (1 << i);
                }
                else
                {
                    uint32_t elapsed = now - (ch_timeout[i] ? ch_timeout[i] : global_start);
                    if (elapsed > 80)
                    {
                        ready_mask |= (1 << i); // 标记为"处理", 但数据无效
                    }
                }
            }
            // 若全部通道都已标记 (就绪或超时), 退出
        }

        // ==========================================================
        // 阶段 3：【极速连读与发送】—— 取回数据并推向 RS485 总线
        // ==========================================================
        for (int i = 0; i < 4; i++)
        {
            float res = -1.0f;

            // 检查刚才的 while 循环中，这个通道是否真的转换成功了
            if ((ready_mask & (1 << i)) != 0)
            {
                res = ADS1220_Fetch_PT1000_Resistance(CS_PINS[i]);
            }

            // 准备串口2 (RS485) 的协议帧缓冲区 (固定 9 字节)
            uint8_t tx_frame[9];
            tx_frame[0] = 0xAA;
            tx_frame[1] = 0x55;
            tx_frame[2] = i + 1;

            if (res > 0)
            {
                // 阻值正常，解算并滤波
                float raw_temp = PT1000_ResistanceToTemperature(res);
                float smooth_temp = Moving_Average_Filter(i, raw_temp);

                // [串口1 DMA]
                sprintf(uart1_tx_buf[i], "CH%d T: %.4f C\r\n", i + 1, smooth_temp);
                HAL_UART_Transmit_DMA(&huart1, (uint8_t *)uart1_tx_buf[i], strlen(uart1_tx_buf[i]));

                // [串口2 RS485] 正常帧装载
                tx_frame[3] = 0x00;
                uint8_t *pTemp = (uint8_t *)&smooth_temp;
                tx_frame[4] = pTemp[0];
                tx_frame[5] = pTemp[1];
                tx_frame[6] = pTemp[2];
                tx_frame[7] = pTemp[3];
            }
            else
            {
                // 超时或未连接
                sprintf(uart1_tx_buf[i], "CH%d Error!\r\n", i + 1);
                HAL_UART_Transmit_DMA(&huart1, (uint8_t *)uart1_tx_buf[i], strlen(uart1_tx_buf[i]));

                // [串口2 RS485] 错误帧装载
                tx_frame[3] = 0x01;
                tx_frame[4] = 0x00;
                tx_frame[5] = 0x00;
                tx_frame[6] = 0x00;
                tx_frame[7] = 0x00;
            }

            // 计算校验和
            uint8_t sum = 0;
            for (int j = 2; j <= 7; j++)
            {
                sum += tx_frame[j];
            }
            tx_frame[8] = sum;

            // 【核心操作】：RS-485 极速阻塞发送
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // TX
            HAL_UART_Transmit(&huart2, tx_frame, 9, 100);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // RX
        }

        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // 用 LED 监控刷新率，现在它会闪得非常快！
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
