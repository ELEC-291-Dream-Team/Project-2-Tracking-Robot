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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEFT_MOTOR_B_Pin GPIO_PIN_0
#define LEFT_MOTOR_B_GPIO_Port GPIOA
#define LEFT_MOTOR_A_Pin GPIO_PIN_1
#define LEFT_MOTOR_A_GPIO_Port GPIOA
#define RIGHT_ADC_Pin GPIO_PIN_2
#define RIGHT_ADC_GPIO_Port GPIOA
#define LEFT_ADC_Pin GPIO_PIN_3
#define LEFT_ADC_GPIO_Port GPIOA
#define WS2812_PIN_Pin GPIO_PIN_1
#define WS2812_PIN_GPIO_Port GPIOB
#define PULSE_IN_Pin GPIO_PIN_7
#define PULSE_IN_GPIO_Port GPIOB
#define RIGHT_MOTOR_B_Pin GPIO_PIN_8
#define RIGHT_MOTOR_B_GPIO_Port GPIOB
#define RIGHT_MOTOR_A_Pin GPIO_PIN_9
#define RIGHT_MOTOR_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define WS2812_HIGH_BIT 60
#define WS2812_LOW_BIT 30
#define LED_COUNT 3

#define ARRAYLEN(x) sizeof(x) / sizeof(x[0])
#define LEFTFORWARD() LeftMotor(0, 1);
#define LEFTREVERSE() LeftMotor(1, 0);
#define LEFTSTOP() LeftMotor(0, 0);
#define RIGHTFORWARD() RightMotor(0, 1);
#define RIGHTREVERSE() RightMotor(1, 0);
#define RIGHTSTOP() RightMotor(0, 0);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
