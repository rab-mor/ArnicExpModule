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
#include "stm32f1xx_hal.h"

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
#define WDOG_Pin GPIO_PIN_3
#define WDOG_GPIO_Port GPIOE
#define KEY_Pin GPIO_PIN_4
#define KEY_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_0
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOC
#define ADC1_Voltage5V_Pin GPIO_PIN_2
#define ADC1_Voltage5V_GPIO_Port GPIOC
#define ADC2_Voltage24V_Pin GPIO_PIN_3
#define ADC2_Voltage24V_GPIO_Port GPIOC
#define ADC2_CurrentSensor6_Pin GPIO_PIN_0
#define ADC2_CurrentSensor6_GPIO_Port GPIOA
#define ADC2_CurrentSensor5_Pin GPIO_PIN_1
#define ADC2_CurrentSensor5_GPIO_Port GPIOA
#define ADC1_CurrentSensor4_Pin GPIO_PIN_2
#define ADC1_CurrentSensor4_GPIO_Port GPIOA
#define ADC1_CurrentSensor3_Pin GPIO_PIN_3
#define ADC1_CurrentSensor3_GPIO_Port GPIOA
#define ADC1_CurrentSensor2_Pin GPIO_PIN_4
#define ADC1_CurrentSensor2_GPIO_Port GPIOA
#define ADC1_CurrentSensor1_Pin GPIO_PIN_5
#define ADC1_CurrentSensor1_GPIO_Port GPIOA
#define ADC1_CurrentSensor9_Pin GPIO_PIN_6
#define ADC1_CurrentSensor9_GPIO_Port GPIOA
#define ADC1_CurrentSensor10_Pin GPIO_PIN_7
#define ADC1_CurrentSensor10_GPIO_Port GPIOA
#define ADC2_CurrentSensor11_Pin GPIO_PIN_4
#define ADC2_CurrentSensor11_GPIO_Port GPIOC
#define ADC2_CurrentSensor12_Pin GPIO_PIN_5
#define ADC2_CurrentSensor12_GPIO_Port GPIOC
#define ADC2_CurrentSensor7_Pin GPIO_PIN_0
#define ADC2_CurrentSensor7_GPIO_Port GPIOB
#define ADC2_CurrentSensor8_Pin GPIO_PIN_1
#define ADC2_CurrentSensor8_GPIO_Port GPIOB
#define RLY1_B_Pin GPIO_PIN_2
#define RLY1_B_GPIO_Port GPIOB
#define RLY1_A_Pin GPIO_PIN_7
#define RLY1_A_GPIO_Port GPIOE
#define RLY2_B_Pin GPIO_PIN_8
#define RLY2_B_GPIO_Port GPIOE
#define RLY2_A_Pin GPIO_PIN_9
#define RLY2_A_GPIO_Port GPIOE
#define RLY3_B_Pin GPIO_PIN_10
#define RLY3_B_GPIO_Port GPIOE
#define RLY3_A_Pin GPIO_PIN_11
#define RLY3_A_GPIO_Port GPIOE
#define RLY4_B_Pin GPIO_PIN_12
#define RLY4_B_GPIO_Port GPIOE
#define RLY4_A_Pin GPIO_PIN_13
#define RLY4_A_GPIO_Port GPIOE
#define RLY8_A_Pin GPIO_PIN_14
#define RLY8_A_GPIO_Port GPIOE
#define RLY8_B_Pin GPIO_PIN_15
#define RLY8_B_GPIO_Port GPIOE
#define RLY7_A_Pin GPIO_PIN_10
#define RLY7_A_GPIO_Port GPIOB
#define RLY7_B_Pin GPIO_PIN_11
#define RLY7_B_GPIO_Port GPIOB
#define RLY6_A_Pin GPIO_PIN_12
#define RLY6_A_GPIO_Port GPIOB
#define RLY6_B_Pin GPIO_PIN_13
#define RLY6_B_GPIO_Port GPIOB
#define RLY5_A_Pin GPIO_PIN_14
#define RLY5_A_GPIO_Port GPIOB
#define RLY5_B_Pin GPIO_PIN_15
#define RLY5_B_GPIO_Port GPIOB
#define RLY12_A_Pin GPIO_PIN_8
#define RLY12_A_GPIO_Port GPIOD
#define RLY12_B_Pin GPIO_PIN_9
#define RLY12_B_GPIO_Port GPIOD
#define RLY11_A_Pin GPIO_PIN_10
#define RLY11_A_GPIO_Port GPIOD
#define RLY11_B_Pin GPIO_PIN_11
#define RLY11_B_GPIO_Port GPIOD
#define RLY10_A_Pin GPIO_PIN_12
#define RLY10_A_GPIO_Port GPIOD
#define RLY10_B_Pin GPIO_PIN_13
#define RLY10_B_GPIO_Port GPIOD
#define RLY9_A_Pin GPIO_PIN_14
#define RLY9_A_GPIO_Port GPIOD
#define RLY9_B_Pin GPIO_PIN_15
#define RLY9_B_GPIO_Port GPIOD
#define UART_EN_Pin GPIO_PIN_8
#define UART_EN_GPIO_Port GPIOA
#define DIP_4_Pin GPIO_PIN_11
#define DIP_4_GPIO_Port GPIOA
#define DIP_3_Pin GPIO_PIN_12
#define DIP_3_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DIP_1_Pin GPIO_PIN_15
#define DIP_1_GPIO_Port GPIOA
#define DIP_2_Pin GPIO_PIN_10
#define DIP_2_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_1
#define KEY2_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOD
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOD
#define D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOD
#define D3_Pin GPIO_PIN_7
#define D3_GPIO_Port GPIOD
#define D2_Pin GPIO_PIN_3
#define D2_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_4
#define D1_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_5
#define D0_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_6
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_7
#define EEPROM_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
