/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTORA_Pin GPIO_PIN_2
#define MOTORA_GPIO_Port GPIOE
#define MOTORB_Pin GPIO_PIN_3
#define MOTORB_GPIO_Port GPIOE
#define DIR4_Pin GPIO_PIN_4
#define DIR4_GPIO_Port GPIOE
#define DIR5_Pin GPIO_PIN_5
#define DIR5_GPIO_Port GPIOE
#define WDFLT1_Pin GPIO_PIN_6
#define WDFLT1_GPIO_Port GPIOE
#define WDFLT2_Pin GPIO_PIN_13
#define WDFLT2_GPIO_Port GPIOC
#define WDFLT3_Pin GPIO_PIN_14
#define WDFLT3_GPIO_Port GPIOC
#define WDFLT4_Pin GPIO_PIN_15
#define WDFLT4_GPIO_Port GPIOC
#define WDFLT5_Pin GPIO_PIN_9
#define WDFLT5_GPIO_Port GPIOF
#define ADC_V5_Pin GPIO_PIN_0
#define ADC_V5_GPIO_Port GPIOC
#define ADC_I1_Pin GPIO_PIN_1
#define ADC_I1_GPIO_Port GPIOC
#define ADC_I2_Pin GPIO_PIN_2
#define ADC_I2_GPIO_Port GPIOC
#define ADC_I3_Pin GPIO_PIN_3
#define ADC_I3_GPIO_Port GPIOC
#define ADC_V1_Pin GPIO_PIN_0
#define ADC_V1_GPIO_Port GPIOA
#define ADC_V2_Pin GPIO_PIN_1
#define ADC_V2_GPIO_Port GPIOA
#define DAC1_Pin GPIO_PIN_2
#define DAC1_GPIO_Port GPIOA
#define ADC_V3_Pin GPIO_PIN_3
#define ADC_V3_GPIO_Port GPIOA
#define DAC_OUT_Pin GPIO_PIN_4
#define DAC_OUT_GPIO_Port GPIOA
#define ADC_IL5_Pin GPIO_PIN_5
#define ADC_IL5_GPIO_Port GPIOA
#define DAC2_Pin GPIO_PIN_6
#define DAC2_GPIO_Port GPIOA
#define ADC_IL1_Pin GPIO_PIN_7
#define ADC_IL1_GPIO_Port GPIOA
#define ADC_IL2_Pin GPIO_PIN_4
#define ADC_IL2_GPIO_Port GPIOC
#define ADC_IL3_Pin GPIO_PIN_5
#define ADC_IL3_GPIO_Port GPIOC
#define ADC_I5_Pin GPIO_PIN_0
#define ADC_I5_GPIO_Port GPIOB
#define DAC3_Pin GPIO_PIN_1
#define DAC3_GPIO_Port GPIOB
#define ADC_IL4_Pin GPIO_PIN_2
#define ADC_IL4_GPIO_Port GPIOB
#define SLEEP1_Pin GPIO_PIN_7
#define SLEEP1_GPIO_Port GPIOE
#define SLEEP2_Pin GPIO_PIN_8
#define SLEEP2_GPIO_Port GPIOE
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOE
#define SLEEP3_Pin GPIO_PIN_10
#define SLEEP3_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOE
#define PWM4_Pin GPIO_PIN_14
#define PWM4_GPIO_Port GPIOE
#define ADC_I4_Pin GPIO_PIN_11
#define ADC_I4_GPIO_Port GPIOB
#define DAC4_Pin GPIO_PIN_12
#define DAC4_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_13
#define CS_GPIO_Port GPIOB
#define ADC_V4_Pin GPIO_PIN_14
#define ADC_V4_GPIO_Port GPIOB
#define FAULT1_Pin GPIO_PIN_15
#define FAULT1_GPIO_Port GPIOB
#define FAULT2_Pin GPIO_PIN_8
#define FAULT2_GPIO_Port GPIOD
#define FAULT3_Pin GPIO_PIN_9
#define FAULT3_GPIO_Port GPIOD
#define FAULT4_Pin GPIO_PIN_10
#define FAULT4_GPIO_Port GPIOD
#define FAULT5_Pin GPIO_PIN_11
#define FAULT5_GPIO_Port GPIOD
#define FAN_Pin GPIO_PIN_12
#define FAN_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOD
#define DAC_IN_Pin GPIO_PIN_14
#define DAC_IN_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOD
#define INT_Pin GPIO_PIN_6
#define INT_GPIO_Port GPIOC
#define STB_Pin GPIO_PIN_7
#define STB_GPIO_Port GPIOC
#define CLK_Pin GPIO_PIN_8
#define CLK_GPIO_Port GPIOC
#define DIO_Pin GPIO_PIN_9
#define DIO_GPIO_Port GPIOC
#define DAC5_Pin GPIO_PIN_8
#define DAC5_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_10
#define PWM3_GPIO_Port GPIOA
#define NRST_OTHER_Pin GPIO_PIN_11
#define NRST_OTHER_GPIO_Port GPIOA
#define SLEEP4_Pin GPIO_PIN_0
#define SLEEP4_GPIO_Port GPIOD
#define SLEEP5_Pin GPIO_PIN_1
#define SLEEP5_GPIO_Port GPIOD
#define CS1_Pin GPIO_PIN_2
#define CS1_GPIO_Port GPIOD
#define PWM5_Pin GPIO_PIN_3
#define PWM5_GPIO_Port GPIOD
#define CS2_Pin GPIO_PIN_4
#define CS2_GPIO_Port GPIOD
#define CS3_Pin GPIO_PIN_6
#define CS3_GPIO_Port GPIOD
#define CS4_Pin GPIO_PIN_7
#define CS4_GPIO_Port GPIOD
#define CS5_Pin GPIO_PIN_6
#define CS5_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_9
#define DIR1_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_0
#define DIR2_GPIO_Port GPIOE
#define DIR3_Pin GPIO_PIN_1
#define DIR3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
