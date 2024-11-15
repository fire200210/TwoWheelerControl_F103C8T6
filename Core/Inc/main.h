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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef bool FlagType;

typedef struct{
  float MotorSpeed;
  float SteeringAngle;
  enum{
    NONE_Control  = 0x00,
    Forward       = 0x01,
    Backward      = 0x02,
    Clockwise     = 0x03,
    Anticlockwise = 0x04,
  }Action;
}ControlINFO_t;

typedef struct{
  uint32_t UART;
  uint32_t PID_Ctrl;
}Count_t;

typedef struct{
  bool PID;
  bool Move, Step;

}Control_flag_t;

enum { MOTOR1, MOTOR2 };
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t Gettime(uint32_t StartTime);
void AT_SEND(UART_HandleTypeDef *huart, uint8_t Aisle, int SendSize);
void SendINFO(UART_HandleTypeDef *huart, const char *format, ...) _ATTRIBUTE ((__format__ (__printf__, 2, 3)));
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define Encoder_CNT             TIM3->CNT
#define Encoder_value_clear     TIM3->CNT = 0
#define Encoder_DIR             __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)

#define Wheel_PPR               260.0f
#define Wheel_Radius_CM         3.35f
#define Wheel_Circumference     21.048670779051614697682f  //(2 * M_PI * Wheel_Radius_CM)

#define _PI_Division_180        0.0174532925199432957692f
#define _180_division_PI        57.295779513082320876798f
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
