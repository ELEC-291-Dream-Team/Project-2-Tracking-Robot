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
#define LED_BUILTIN_Pin GPIO_PIN_13
#define LED_BUILTIN_GPIO_Port GPIOC
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
#define SONAR_ECHO_Pin GPIO_PIN_8
#define SONAR_ECHO_GPIO_Port GPIOA
#define SONAR_TRIGGER_Pin GPIO_PIN_11
#define SONAR_TRIGGER_GPIO_Port GPIOA
#define BUZZER_PIN_Pin GPIO_PIN_6
#define BUZZER_PIN_GPIO_Port GPIOB
#define PULSE_IN_Pin GPIO_PIN_7
#define PULSE_IN_GPIO_Port GPIOB
#define RIGHT_MOTOR_B_Pin GPIO_PIN_8
#define RIGHT_MOTOR_B_GPIO_Port GPIOB
#define RIGHT_MOTOR_A_Pin GPIO_PIN_9
#define RIGHT_MOTOR_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TARGETADCLEFT 700
#define TARGETADCRIGHT 1050
#define DEADZONE 150
#define WS2812_HIGH_BIT 60
#define WS2812_LOW_BIT 30
#define LED_COUNT 3

#define ARRAYLEN(x) sizeof(x) / sizeof(x[0])
#define COMPARE(variable, value, tolerance) variable > value - tolerance && variable < value + tolerance
#define LEFTFORWARD() LeftMotor(0, 1);
#define LEFTREVERSE() LeftMotor(1, 0);
#define LEFTSTOP() LeftMotor(0, 0);
#define RIGHTFORWARD() RightMotor(0, 1);
#define RIGHTREVERSE() RightMotor(1, 0);
#define RIGHTSTOP() RightMotor(0, 0);

#define LEFTADC ADCBuffer[0]
#define RIGHTADC ADCBuffer[1]
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
