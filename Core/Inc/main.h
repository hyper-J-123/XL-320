/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define DIR1_Pin GPIO_PIN_1
#define DIR1_GPIO_Port GPIOA
#define DIR2_Pin GPIO_PIN_4
#define DIR2_GPIO_Port GPIOB

// XL 系列舵机控制表地址
#define XL320Led                 0x19
#define XL320Torque              0x18
#define XL320LPosition           0x1e

#define XL430Led                 0x41
#define XL430Torque              0x40
#define XL430LPosition           0x74

#define xl430Step                0.08789f
#define xl320Step                0.29f

// 舵机ID定义
#define SERVO_ID_1               0x01
#define SERVO_ID_2               0x02
#define SERVO_ID               	 0x00

// XL-320 LED 颜色定义
#define LED_OFF                  0
#define LED_RED                  1
#define LED_GREEN                2
#define LED_YELLOW				 3
#define LED_BLUE                 4
#define LED_PURPLE				 5
#define LED_CYAN				 6
#define LED_WHITE				 7

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
