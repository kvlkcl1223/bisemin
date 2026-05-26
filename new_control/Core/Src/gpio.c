/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DIR4_Pin|DIR5_Pin|SLEEP1_Pin|SLEEP2_Pin
                          |SLEEP3_Pin|DIR2_Pin|DIR3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|CS5_Pin|DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|SLEEP4_Pin|SLEEP5_Pin
                          |CS1_Pin|CS2_Pin|CS3_Pin|CS4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STB_Pin|CLK_Pin|DIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRST_OTHER_GPIO_Port, NRST_OTHER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR4_Pin DIR5_Pin SLEEP1_Pin SLEEP2_Pin
                           SLEEP3_Pin DIR2_Pin DIR3_Pin */
  GPIO_InitStruct.Pin = DIR4_Pin|DIR5_Pin|SLEEP1_Pin|SLEEP2_Pin
                          |SLEEP3_Pin|DIR2_Pin|DIR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : WDFLT1_Pin */
  GPIO_InitStruct.Pin = WDFLT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WDFLT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WDFLT2_Pin WDFLT3_Pin WDFLT4_Pin INT_Pin */
  GPIO_InitStruct.Pin = WDFLT2_Pin|WDFLT3_Pin|WDFLT4_Pin|INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : WDFLT5_Pin */
  GPIO_InitStruct.Pin = WDFLT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WDFLT5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin CS5_Pin DIR1_Pin */
  GPIO_InitStruct.Pin = CS_Pin|CS5_Pin|DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT1_Pin */
  GPIO_InitStruct.Pin = FAULT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAULT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FAULT2_Pin FAULT3_Pin FAULT4_Pin FAULT5_Pin */
  GPIO_InitStruct.Pin = FAULT2_Pin|FAULT3_Pin|FAULT4_Pin|FAULT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin SLEEP4_Pin SLEEP5_Pin
                           CS1_Pin CS2_Pin CS3_Pin CS4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|SLEEP4_Pin|SLEEP5_Pin
                          |CS1_Pin|CS2_Pin|CS3_Pin|CS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : STB_Pin CLK_Pin DIO_Pin */
  GPIO_InitStruct.Pin = STB_Pin|CLK_Pin|DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NRST_OTHER_Pin */
  GPIO_InitStruct.Pin = NRST_OTHER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRST_OTHER_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
