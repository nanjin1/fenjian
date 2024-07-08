/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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
#define US1_Pin GPIO_PIN_2
#define US1_GPIO_Port GPIOE
#define US2_Pin GPIO_PIN_0
#define US2_GPIO_Port GPIOC
#define MOTOR1_ENCODER_Pin GPIO_PIN_1
#define MOTOR1_ENCODER_GPIO_Port GPIOA
#define MOTOR2_ENCODER_Pin GPIO_PIN_3
#define MOTOR2_ENCODER_GPIO_Port GPIOA
#define MOTOR3_ENCODER_Pin GPIO_PIN_7
#define MOTOR3_ENCODER_GPIO_Port GPIOA
#define MOTOR4_ENCODER_Pin GPIO_PIN_1
#define MOTOR4_ENCODER_GPIO_Port GPIOB
#define US_SEND_Pin GPIO_PIN_12
#define US_SEND_GPIO_Port GPIOB
#define US_RECEIVE_Pin GPIO_PIN_13
#define US_RECEIVE_GPIO_Port GPIOB
#define US_RECEIVE_EXTI_IRQn EXTI15_10_IRQn
#define SW_1_Pin GPIO_PIN_14
#define SW_1_GPIO_Port GPIOB
#define SW_2_Pin GPIO_PIN_15
#define SW_2_GPIO_Port GPIOB
#define SW_3_Pin GPIO_PIN_10
#define SW_3_GPIO_Port GPIOD
#define HW_Circle1_Pin GPIO_PIN_12
#define HW_Circle1_GPIO_Port GPIOD
#define HW_Circle2_Pin GPIO_PIN_13
#define HW_Circle2_GPIO_Port GPIOD
#define HW_Circle3_Pin GPIO_PIN_14
#define HW_Circle3_GPIO_Port GPIOD
#define HW_Height1_Pin GPIO_PIN_15
#define HW_Height1_GPIO_Port GPIOD
#define HW_Height2_Pin GPIO_PIN_8
#define HW_Height2_GPIO_Port GPIOC
#define SW_7_Pin GPIO_PIN_9
#define SW_7_GPIO_Port GPIOC
#define SW_8_Pin GPIO_PIN_8
#define SW_8_GPIO_Port GPIOA
#define FengFeng_Pin GPIO_PIN_12
#define FengFeng_GPIO_Port GPIOA
#define HW_S1_Pin GPIO_PIN_15
#define HW_S1_GPIO_Port GPIOA
#define HW_S2_Pin GPIO_PIN_0
#define HW_S2_GPIO_Port GPIOD
#define HW_S3_Pin GPIO_PIN_1
#define HW_S3_GPIO_Port GPIOD
#define HW_S4_Pin GPIO_PIN_3
#define HW_S4_GPIO_Port GPIOD
#define Side_HW1_Pin GPIO_PIN_5
#define Side_HW1_GPIO_Port GPIOB
#define Side_HW2_Pin GPIO_PIN_9
#define Side_HW2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
