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
#include "stdbool.h"
#include "sys.h"
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
// XL系列舵机函数声明
void xlSeriesStart(void);
void xlSeriesControlMode(uint8_t id, uint8_t mode);
void xlSeriesSetDirection(uint8_t tx_mode);
void xlSeriesSendFrame(UART_HandleTypeDef *huart, uint8_t *frame, uint16_t length);
uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
void xlSeriesLed(uint8_t id, uint8_t on, uint8_t address);
void xlSeriesTorque(uint8_t id, uint8_t on, uint8_t address);
void xl320SendPosition(uint8_t id, uint16_t position);
void xl320SendMovingSpeed(uint8_t id, uint16_t movingSpeed);
void xl320SendPGain(uint8_t id, uint8_t pGain);
uint16_t xl320ReadPosition(uint8_t id);
void xlPowerOff(uint8_t isOn);
//void xl320ReadPosition(uint8_t id);
uint8_t verifyPositionPacket(uint8_t *data);
uint16_t parsePositionValue(uint8_t *data);
uint16_t processPositionData(void);
void USART2_ReadCallback(void);
uint16_t ReadPositionAndSendToPC(uint8_t id);
void  Read9ServosAndSend_Protocol(void);
void Control9Servos(void);
uint8_t xl320CheckMovingStatus(uint8_t id);
void TestAccuracy(uint8_t id, uint16_t move_units);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
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
#define SERVO_ID_3               0x03
#define SERVO_ID_4               0x04
#define SERVO_ID_5               0x05
#define SERVO_ID_6               0x06
#define SERVO_ID_7               0x07
#define SERVO_ID_8               0x08
#define SERVO_ID_9               0x09
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


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
