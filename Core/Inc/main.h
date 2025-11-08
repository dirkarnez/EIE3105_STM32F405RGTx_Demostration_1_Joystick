/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define MOTOR_EN_Pin GPIO_PIN_13
#define MOTOR_EN_GPIO_Port GPIOC
#define ENCODER1_A_Pin GPIO_PIN_0
#define ENCODER1_A_GPIO_Port GPIOA
#define ENCODER1_B_Pin GPIO_PIN_1
#define ENCODER1_B_GPIO_Port GPIOA
#define ENCODER2_A_Pin GPIO_PIN_5
#define ENCODER2_A_GPIO_Port GPIOA
#define ADC_V_Pin GPIO_PIN_4
#define ADC_V_GPIO_Port GPIOC
#define ADC_A_Pin GPIO_PIN_5
#define ADC_A_GPIO_Port GPIOC
#define PWM2_B_Pin GPIO_PIN_14
#define PWM2_B_GPIO_Port GPIOB
#define PWM2_A_Pin GPIO_PIN_15
#define PWM2_A_GPIO_Port GPIOB
#define ST_Link_DIO_Pin GPIO_PIN_13
#define ST_Link_DIO_GPIO_Port GPIOA
#define ST_Link_CLK_Pin GPIO_PIN_14
#define ST_Link_CLK_GPIO_Port GPIOA
#define ENCODER2_B_Pin GPIO_PIN_3
#define ENCODER2_B_GPIO_Port GPIOB
#define WS2812_Pin GPIO_PIN_5
#define WS2812_GPIO_Port GPIOB
#define OLED_I2C_SCL_Pin GPIO_PIN_6
#define OLED_I2C_SCL_GPIO_Port GPIOB
#define OLED_I2C_SDA_Pin GPIO_PIN_7
#define OLED_I2C_SDA_GPIO_Port GPIOB
#define PWM1_B_Pin GPIO_PIN_8
#define PWM1_B_GPIO_Port GPIOB
#define PWM1_A_Pin GPIO_PIN_9
#define PWM1_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
