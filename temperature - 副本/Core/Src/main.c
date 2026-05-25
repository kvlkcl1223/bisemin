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
// ���ź궨�� (�����ʵ������޸�)
#define ADS1220_CS_LOW()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define ADS1220_CS_HIGH()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define ADS1220_DRDY_READ() HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)

// ADS1220 ����ָ��
#define CMD_RESET       0x06
#define CMD_START_SYNC  0x08
#define CMD_RDATA       0x10
#define FILTER_N 10
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
/**
  * @brief  SPI �����շ�����
  */
uint8_t ADS1220_SPI_Transfer(uint8_t data) {
    uint8_t rxData = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rxData, 1, HAL_MAX_DELAY);
    return rxData;
}

/**
  * @brief  д ADS1220 �Ĵ���
  */
void ADS1220_WriteReg(uint8_t regAddr, uint8_t data) {
    ADS1220_CS_LOW();
    ADS1220_SPI_Transfer(0x40 | (regAddr << 2)); 
    ADS1220_SPI_Transfer(data);
    ADS1220_CS_HIGH();
}

/**
  * @brief  ��ʼ�� ADS1220 (��� PT100, R_REF=4k��, IDAC=500uA, Gain=16)
  */
void ADS1220_Init(void) {
    ADS1220_CS_LOW();
    ADS1220_SPI_Transfer(CMD_RESET);
    ADS1220_CS_HIGH();
    HAL_Delay(50); // �ȴ���λ���

    // �Ĵ��� 0: AIN1��AIN2���, ����=16, PGA����
    ADS1220_WriteReg(0x00, 0x38); 
    // �Ĵ��� 1: �������� 20SPS, ����ת��ģʽ
    ADS1220_WriteReg(0x01, 0x04); 
    // �Ĵ��� 2: �ⲿ�ο� REFP0/REFN0, IDAC=500uA
    ADS1220_WriteReg(0x02, 0x5C); 
    // �Ĵ��� 3: ����Դ IDAC1 ·�ɵ� AIN0
    ADS1220_WriteReg(0x03, 0x20); 

    // ����ת��
    ADS1220_CS_LOW();
    ADS1220_SPI_Transfer(CMD_START_SYNC);
    ADS1220_CS_HIGH();
}

/**
  * @brief  ��ȡ 24λ ADC ������ PT100 ��ֵ
  */
float ADS1220_Read_PT100_Resistance(void) {
    uint32_t rawData = 0;
    int32_t signedData = 0;

    // �ȴ� DRDY ���ű� ADS1220 ����
    uint32_t timeout = HAL_GetTick();
    while(ADS1220_DRDY_READ() == GPIO_PIN_SET) {
        if((HAL_GetTick() - timeout) > 200) return -1.0; // ��ʱ����
    }

    ADS1220_CS_LOW();
    ADS1220_SPI_Transfer(CMD_RDATA);
    rawData |= ADS1220_SPI_Transfer(0xFF) << 16;
    rawData |= ADS1220_SPI_Transfer(0xFF) << 8;
    rawData |= ADS1220_SPI_Transfer(0xFF);
    ADS1220_CS_HIGH();

    // ����ת��
    if(rawData & 0x800000) {
        signedData = (int32_t)(rawData | 0xFF000000); 
    } else {
        signedData = (int32_t)rawData;
    }

    // ��Ļ�׼������ 4000 ŷķ
    float R_REF = 4000.0f; 
    
    // ����������ʽ����
    return ((float)signedData * R_REF) / (16.0f * 8388608.0f);
}

/**
  * @brief  �߾�����ֵת�¶��㷨
  */
float PT100_ResistanceToTemperature(float resistance) {
    float R0 = 100.0f;
    float A = 3.9083e-3f;
    float B = -5.775e-7f;
    
    if (resistance < 100.0f) {
        return (resistance - 100.0f) / 0.3851f; 
    } else {
        return (-R0*A + sqrtf((R0*A)*(R0*A) - 4*R0*B*(R0 - resistance))) / (2*R0*B);
    }
}
float Moving_Average_Filter(float new_temp) {
    static float temp_buffer[FILTER_N] = {0}; // 静态数组做环形队列
    static uint8_t index = 0;
    static uint8_t is_full = 0;
    float sum = 0.0f;
    
    // 存入新数据
    temp_buffer[index] = new_temp;
    index++;
    
    if (index >= FILTER_N) {
        index = 0;
        is_full = 1;
    }
    
    // 如果缓冲区还没填满，就直接返回当前值
    if (!is_full) {
        return new_temp;
    }
    
    // 缓冲区满了，计算平均值
    for (uint8_t i = 0; i < FILTER_N; i++) {
        sum += temp_buffer[i];
    }
    
    return sum / FILTER_N;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//char uart_buf[100]; // ׼��һ���㹻����ַ�������Ϊ���ͻ�����

//    // ��ʽ��������Ϣ������
//    sprintf(uart_buf, "==================================\r\n");
//    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
//    
//    sprintf(uart_buf, "ADS1220 Started! R_REF = 4kOhm\r\n");
//    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

//    // ��ʼ�� ADS1220
//    ADS1220_Init();


char uart_buf[100];
    sprintf(uart_buf, "==================================\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);

    // 1. ��ʼ�� ADS1220
    ADS1220_Init();
    
    // 2. ǿ�ж�ȡ���Ǹղ�д��ȥ�ļĴ��� 0
    uint8_t read_reg0 = 0;
    
    ADS1220_CS_LOW();
    ADS1220_SPI_Transfer(0x20); // ������: 0010 0000 (���Ĵ���0)
    read_reg0 = ADS1220_SPI_Transfer(0xFF); // ���Ϳ�ʱ�Ӷ�ȡ����
    ADS1220_CS_HIGH();

    // 3. ��ӡ������ֵ
    sprintf(uart_buf, "Read Reg 0: 0x%02X\r\n", read_reg0);
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// 1. ��ȡ����ֵ
        float res = ADS1220_Read_PT100_Resistance();
        
        if (res > 0) {
            // 2. �����¶�
            //float temp = PT100_ResistanceToTemperature(res);
            float raw_temp = PT100_ResistanceToTemperature(res);
						float smooth_temp = Moving_Average_Filter(raw_temp);
            // 3. ����������ʽ���������� (����4λС��)
            sprintf(uart_buf, "R: %.4f Ohm  |  T: %.4f C\r\n", res, smooth_temp);
            
            // 4. ����ԭ�� HAL �⺯�����ͳ�ȥ
            HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
            
        } else {
            // ��ʱ������
            sprintf(uart_buf, "Error: ADS1220 Timeout or Not Connected!\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
        }

        // �ȴ� 500ms ������һ�β���
        HAL_Delay(500);
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
