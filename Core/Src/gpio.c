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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, WDOG_Pin|RLY1_A_Pin|RLY2_B_Pin|RLY2_A_Pin
                          |RLY3_B_Pin|RLY3_A_Pin|RLY4_B_Pin|RLY4_A_Pin
                          |RLY8_A_Pin|RLY8_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_R_Pin|LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RLY1_B_Pin|RLY7_A_Pin|RLY7_B_Pin|RLY6_A_Pin
                          |RLY6_B_Pin|RLY5_A_Pin|RLY5_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RLY12_A_Pin|RLY12_B_Pin|RLY11_A_Pin|RLY11_B_Pin
                          |RLY10_A_Pin|RLY10_B_Pin|RLY9_A_Pin|RLY9_B_Pin
                          |LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WDOG_Pin RLY1_A_Pin RLY2_B_Pin RLY2_A_Pin
                           RLY3_B_Pin RLY3_A_Pin RLY4_B_Pin RLY4_A_Pin
                           RLY8_A_Pin RLY8_B_Pin */
  GPIO_InitStruct.Pin = WDOG_Pin|RLY1_A_Pin|RLY2_B_Pin|RLY2_A_Pin
                          |RLY3_B_Pin|RLY3_A_Pin|RLY4_B_Pin|RLY4_A_Pin
                          |RLY8_A_Pin|RLY8_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RLY1_B_Pin RLY7_A_Pin RLY7_B_Pin RLY6_A_Pin
                           RLY6_B_Pin RLY5_A_Pin RLY5_B_Pin */
  GPIO_InitStruct.Pin = RLY1_B_Pin|RLY7_A_Pin|RLY7_B_Pin|RLY6_A_Pin
                          |RLY6_B_Pin|RLY5_A_Pin|RLY5_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RLY12_A_Pin RLY12_B_Pin RLY11_A_Pin RLY11_B_Pin
                           RLY10_A_Pin RLY10_B_Pin RLY9_A_Pin RLY9_B_Pin
                           LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = RLY12_A_Pin|RLY12_B_Pin|RLY11_A_Pin|RLY11_B_Pin
                          |RLY10_A_Pin|RLY10_B_Pin|RLY9_A_Pin|RLY9_B_Pin
                          |LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_EN_Pin */
  GPIO_InitStruct.Pin = UART_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP_4_Pin DIP_3_Pin DIP_1_Pin */
  GPIO_InitStruct.Pin = DIP_4_Pin|DIP_3_Pin|DIP_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIP_2_Pin */
  GPIO_InitStruct.Pin = DIP_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIP_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY2_Pin KEY1_Pin D5_Pin D4_Pin
                           D3_Pin */
  GPIO_InitStruct.Pin = KEY2_Pin|KEY1_Pin|D5_Pin|D4_Pin
                          |D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_Pin D1_Pin D0_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D1_Pin|D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
