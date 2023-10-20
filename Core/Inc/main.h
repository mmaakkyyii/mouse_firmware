/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOC
#define LED_L_Pin GPIO_PIN_14
#define LED_L_GPIO_Port GPIOC
#define LED_FL_Pin GPIO_PIN_15
#define LED_FL_GPIO_Port GPIOC
#define SENSOR_FR_Pin GPIO_PIN_0
#define SENSOR_FR_GPIO_Port GPIOA
#define SENSOR_R_Pin GPIO_PIN_1
#define SENSOR_R_GPIO_Port GPIOA
#define SENSOR_L_Pin GPIO_PIN_2
#define SENSOR_L_GPIO_Port GPIOA
#define SENSOR_FL_Pin GPIO_PIN_3
#define SENSOR_FL_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_2
#define LED4_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOB
#define BAT_ADC_Pin GPIO_PIN_11
#define BAT_ADC_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_8
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_9
#define AIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_10
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_11
#define BIN2_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_12
#define SW2_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_10
#define SW1_GPIO_Port GPIOC
#define LED_FR_Pin GPIO_PIN_9
#define LED_FR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
