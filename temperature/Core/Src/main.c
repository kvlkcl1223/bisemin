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
const uint16_t CS_PINS[4]   = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
const uint16_t DRDY_PINS[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10};

#define CMD_RESET       0x06
#define CMD_START_SYNC  0x08
#define CMD_RDATA       0x10
#define FILTER_N        10

// ---------------- 2. 底层 SPI 与安全互斥片选 ----------------
void Safe_CS_Select(uint16_t target_cs_pin) {
    // 强制拉高(关闭)所有通道，防止总线打架
    for(int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(GPIOB, CS_PINS[i], GPIO_PIN_SET);
    }
    // 拉低(打开)当前需要的通道
    HAL_GPIO_WritePin(GPIOB, target_cs_pin, GPIO_PIN_RESET);
}

void Safe_CS_Deselect(uint16_t target_cs_pin) {
    HAL_GPIO_WritePin(GPIOB, target_cs_pin, GPIO_PIN_SET);
}

uint8_t ADS1220_SPI_Transfer(uint8_t data) {
    uint8_t rxData = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rxData, 1, 100);
    return rxData;
}

void ADS1220_WriteReg(uint16_t cs_pin, uint8_t regAddr, uint8_t data) {
    Safe_CS_Select(cs_pin);
    ADS1220_SPI_Transfer(0x40 | (regAddr << 2)); 
    ADS1220_SPI_Transfer(data);
    Safe_CS_Deselect(cs_pin);
}

// ---------------- 3. 初始化与读取逻辑 ----------------


// ---------------- 4. 算法部分 ----------------
float PT1000_ResistanceToTemperature(float resistance) {
    float R0 = 1000.0f;
    float A = 3.9083e-3f;
    float B = -5.775e-7f;
    
    if (resistance < 1000.0f) {
        return (resistance - 1000.0f) / 3.851f; 
    } else {
        return (-R0*A + sqrtf((R0*A)*(R0*A) - 4*R0*B*(R0 - resistance))) / (2*R0*B);
    }
}

float Moving_Average_Filter(uint8_t ch_idx, float new_temp) {
    static float temp_buffer[4][FILTER_N] = {0}; // 4路独立的滤波池
    static uint8_t index[4] = {0};               
    static uint8_t is_full[4] = {0};             
    float sum = 0.0f;
    
    temp_buffer[ch_idx][index[ch_idx]] = new_temp;
    index[ch_idx]++;
    if (index[ch_idx] >= FILTER_N) {
        index[ch_idx] = 0;
        is_full[ch_idx] = 1;
    }
    
    if (!is_full[ch_idx]) return new_temp;
    
    for (uint8_t i = 0; i < FILTER_N; i++) {
        sum += temp_buffer[ch_idx][i];
    }
    return sum / FILTER_N;
}
/**
  * @brief  初始化特定通道的 ADS1220 (完美适配 PT1000)
  */
void ADS1220_Init(uint16_t cs_pin) {
    // 软件复位
    Safe_CS_Select(cs_pin);
    ADS1220_SPI_Transfer(CMD_RESET);
    Safe_CS_Deselect(cs_pin);
    HAL_Delay(5); 

    // 【修复1：Reg 0】 0x32 -> 测 AIN1-AIN2, 增益2倍, PGA开启
    ADS1220_WriteReg(cs_pin, 0x00, 0x32); 
    // 【Reg 1】 0x00 -> 20SPS, 单次转换模式
    ADS1220_WriteReg(cs_pin, 0x01, 0x00); 
    // 【修复2：Reg 2】 0x53 -> 外部参考, 开启50Hz滤波, IDAC精准250uA
    ADS1220_WriteReg(cs_pin, 0x02, 0x53); 
    // 【Reg 3】 0x20 -> 电流从 AIN0 吐出
    ADS1220_WriteReg(cs_pin, 0x03, 0x20); 
}

/**
  * @brief  读取 24位 ADC 并解算阻值 (防死锁安全版)
  */
float ADS1220_Read_PT1000_Resistance(uint16_t cs_pin, uint16_t drdy_pin) {
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
    while(HAL_GPIO_ReadPin(GPIOB, drdy_pin) == GPIO_PIN_SET) {
        if((HAL_GetTick() - start_time) > 150) return -1.0; // 超时保护
    }

    // 3. 读取 24 位数据
    Safe_CS_Select(cs_pin);
    ADS1220_SPI_Transfer(CMD_RDATA);
    rawData |= ADS1220_SPI_Transfer(0xFF) << 16;
    rawData |= ADS1220_SPI_Transfer(0xFF) << 8;
    rawData |= ADS1220_SPI_Transfer(0xFF);
    Safe_CS_Deselect(cs_pin);

    // 4. 补码换算
    if(rawData & 0x800000) {
        signedData = (int32_t)(rawData | 0xFF000000); 
    } else {
        signedData = (int32_t)rawData;
    }

    // 换算公式: 基准电阻 4k, 增益 2 倍
    return ((float)signedData * 4000.0f) / (2.0f * 8388608.0f);
}

/**
  * @brief  发送命令：并发触发转换专用
  */
void ADS1220_Start_Conversion(uint16_t cs_pin) {
    Safe_CS_Select(cs_pin);
    ADS1220_SPI_Transfer(CMD_START_SYNC);
    Safe_CS_Deselect(cs_pin);
}

/**
  * @brief  纯净读取函数：不包含等待，直接去 SPI 总线拿 24 位数据
  */
float ADS1220_Fetch_PT1000_Resistance(uint16_t cs_pin) {
    uint32_t rawData = 0;
    int32_t signedData = 0;

    Safe_CS_Select(cs_pin);
    ADS1220_SPI_Transfer(CMD_RDATA);
    rawData |= ADS1220_SPI_Transfer(0xFF) << 16;
    rawData |= ADS1220_SPI_Transfer(0xFF) << 8;
    rawData |= ADS1220_SPI_Transfer(0xFF);
    Safe_CS_Deselect(cs_pin);

    // 补码换算
    if(rawData & 0x800000) {
        signedData = (int32_t)(rawData | 0xFF000000); 
    } else {
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
  for(int i = 0; i < 4; i++) {
      HAL_GPIO_WritePin(GPIOB, CS_PINS[i], GPIO_PIN_SET);
  }
  HAL_Delay(100);

  // 2. 依次初始化 4 个通道的 ADS1220
  for(int i = 0; i < 4; i++) {
      ADS1220_Init(CS_PINS[i]);
      HAL_Delay(10);
  }
  
  sprintf(uart_buf, "4-CH PT1000 Initialization OK!\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//	
//char uart_buf[100];
//  sprintf(uart_buf, "==================================\r\n");
//  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);

//  // 初始化四路 ADS1220
//  for(int i = 0; i < 4; i++) {
//      // 在初始化之前，为了安全起见，先拉高所有 CS 引脚，防止总线踩踏
//      HAL_GPIO_WritePin(GPIOB, CS_PINS[i], GPIO_PIN_SET); 
//  }
//  
//  for(int i = 0; i < 4; i++) {
//      ADS1220_Init(CS_PINS[i]);
//      HAL_Delay(10); // 给每个芯片一点准备时间
//  }
//  
//  sprintf(uart_buf, "4-Channel ADS1220 Init OK!\r\n");
//  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//// 为了防止 DMA 发送时内存被覆盖，定义一个二维数组，为每个通道提供独立的发送缓冲池
//    static char uart1_tx_buf[4][100]; 

//    // 轮流读取四个通道
//    for(int i = 0; i < 4; i++) {
//        // 读取阻值
//        float res = ADS1220_Read_PT1000_Resistance(CS_PINS[i], DRDY_PINS[i]);
//        
//        // 准备串口2 (RS485) 的协议帧缓冲区 (固定 9 字节)
//        uint8_t tx_frame[9];
//        tx_frame[0] = 0xAA;      // 帧头1
//        tx_frame[1] = 0x55;      // 帧头2
//        tx_frame[2] = i + 1;     // 通道号 (1~4)
//        
//        if (res > 0) {
//            // 阻值正常，计算真实温度并滤波
//            float raw_temp = PT1000_ResistanceToTemperature(res);
//            float smooth_temp = Moving_Average_Filter(i, raw_temp);
//            
//            // 1. [串口1 DMA 发送] 组装调试字符串并启动后台 DMA (非阻塞)
//            sprintf(uart1_tx_buf[i], "CH%d T: %.4f C\r\n", i+1, smooth_temp);
//            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)uart1_tx_buf[i], strlen(uart1_tx_buf[i]));
//            
//            // 2. [串口2 RS485 发送] 组装 16 进制协议数据
//            tx_frame[3] = 0x00; // 状态：正常
//            
//            // 强转提取 float 的 4 个字节内存
//            uint8_t *pTemp = (uint8_t *)&smooth_temp;
//            tx_frame[4] = pTemp[0];
//            tx_frame[5] = pTemp[1];
//            tx_frame[6] = pTemp[2];
//            tx_frame[7] = pTemp[3];
//            
//        } else {
//            // 如果某一路探头超时或未连接
//            sprintf(uart1_tx_buf[i], "CH%d Error!\r\n", i+1);
//            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)uart1_tx_buf[i], strlen(uart1_tx_buf[i]));
//            
//            // 组装错误帧
//            tx_frame[3] = 0x01; // 状态：错误
//            tx_frame[4] = 0x00; 
//            tx_frame[5] = 0x00;
//            tx_frame[6] = 0x00;
//            tx_frame[7] = 0x00;
//        }
//        
//        // 计算 RS485 帧的校验和 (Byte 2 到 Byte 7)
//        uint8_t sum = 0;
//        for(int j = 2; j <= 7; j++) {
//            sum += tx_frame[j];
//        }
//        tx_frame[8] = sum;
//        
//        // ------------------------------------------------------------
//        // 【核心操作】：RS-485 极速阻塞发送
//        // ------------------------------------------------------------
//        // 1. 将 PA1 拉高，把 MAX3485 切换为发送模式 (TX Enable)
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//        
//        // 2. 阻塞式发送 9 个字节 (它会自动等待所有字节彻底发完)
//        HAL_UART_Transmit(&huart2, tx_frame, 9, 100);
//        
//        // 3. 极其关键！发完瞬间将 PA1 拉低，立刻释放总线，切回接收模式 (RX Enable)
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//        // ------------------------------------------------------------
//    }
//		
//			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
////    // (可选) 如果你希望每次 4 路读完后，串口 1 打印一个分割线
////    // 这里不再使用 DMA，为了防止与上面的 DMA 抢占资源，直接用阻塞发送
////    char split_str[] = "----------------------------------\r\n";
////    HAL_UART_Transmit(&huart1, (uint8_t *)split_str, strlen(split_str), 100);
//		
	static char uart1_tx_buf[4][100]; 

    // ==========================================================
    // 阶段 1：【并发触发】—— 一口令下，4个ADC同时开始物理转换
    // ==========================================================
    for(int i = 0; i < 4; i++) {
        ADS1220_Start_Conversion(CS_PINS[i]);
    }

    // 给芯片拉高 DRDY 的反应时间
    HAL_Delay(2); 

    // ==========================================================
    // 阶段 2：【统一等待】—— 死等 4 个通道的 DRDY 全部被拉低
    // ==========================================================
    uint32_t start_time = HAL_GetTick();
    uint8_t ready_mask = 0x00; // 用低 4 位记录哪个通道准备好了

    // 当 ready_mask 变成 0x0F (即 0000 1111) 时，说明 4 个都好了
    while(ready_mask != 0x0F) {
        ready_mask = 0;
        if(HAL_GPIO_ReadPin(GPIOB, DRDY_PINS[0]) == GPIO_PIN_RESET) ready_mask |= 0x01;
        if(HAL_GPIO_ReadPin(GPIOB, DRDY_PINS[1]) == GPIO_PIN_RESET) ready_mask |= 0x02;
        if(HAL_GPIO_ReadPin(GPIOB, DRDY_PINS[2]) == GPIO_PIN_RESET) ready_mask |= 0x04;
        if(HAL_GPIO_ReadPin(GPIOB, DRDY_PINS[3]) == GPIO_PIN_RESET) ready_mask |= 0x08;

        if((HAL_GetTick() - start_time) > 150) {
            break; // 防止某个探头断开导致全盘死锁，超时强制退出
        }
    }

    // ==========================================================
    // 阶段 3：【极速连读与发送】—— 取回数据并推向 RS485 总线
    // ==========================================================
    for(int i = 0; i < 4; i++) {
        float res = -1.0f;
        
        // 检查刚才的 while 循环中，这个通道是否真的转换成功了
        if ((ready_mask & (1 << i)) != 0) {
            res = ADS1220_Fetch_PT1000_Resistance(CS_PINS[i]);
        }

        // 准备串口2 (RS485) 的协议帧缓冲区 (固定 9 字节)
        uint8_t tx_frame[9];
        tx_frame[0] = 0xAA;      
        tx_frame[1] = 0x55;      
        tx_frame[2] = i + 1;     
        
        if (res > 0) {
            // 阻值正常，解算并滤波
            float raw_temp = PT1000_ResistanceToTemperature(res);
            float smooth_temp = Moving_Average_Filter(i, raw_temp);
            
            // [串口1 DMA] 
            sprintf(uart1_tx_buf[i], "CH%d T: %.4f C\r\n", i+1, smooth_temp);
            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)uart1_tx_buf[i], strlen(uart1_tx_buf[i]));
            
            // [串口2 RS485] 正常帧装载
            tx_frame[3] = 0x00; 
            uint8_t *pTemp = (uint8_t *)&smooth_temp;
            tx_frame[4] = pTemp[0];
            tx_frame[5] = pTemp[1];
            tx_frame[6] = pTemp[2];
            tx_frame[7] = pTemp[3];
            
        } else {
            // 超时或未连接
            sprintf(uart1_tx_buf[i], "CH%d Error!\r\n", i+1);
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
        for(int j = 2; j <= 7; j++) {
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
