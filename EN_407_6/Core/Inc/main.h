/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;
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
#define Motor_L1_Pin GPIO_PIN_3
#define Motor_L1_GPIO_Port GPIOA
#define Motor_L2_Pin GPIO_PIN_4
#define Motor_L2_GPIO_Port GPIOA
#define Motor_R1_Pin GPIO_PIN_4
#define Motor_R1_GPIO_Port GPIOC
#define Motor_R2_Pin GPIO_PIN_5
#define Motor_R2_GPIO_Port GPIOC


//圆筒电机定时器
#define SERVO_BACK_GRAP_TIM htim5
#define SERVO_BACK_GRAP_OPEN_ANGLE 35



//后侧电机左侧夹爪电机定时器通道 -- 通道1
#define SERVO_BACK_GRAP_L_CHANNEL TIM_CHANNEL_1
//左侧电即关闭角度
#define SERVO_BACK_GRAP_L_CLOSE_ANGLE 180
//左侧电机张开角度
#define SERVO_BACK_GRAP_L_OPEN_ANGLE (SERVO_BACK_GRAP_L_CLOSE_ANGLE - SERVO_BACK_GRAP_OPEN_ANGLE)

//后侧电机右侧夹爪电机定时器通道 -- 通道2
#define SERVO_BACK_GRAP_R_CHANNEL TIM_CHANNEL_2
//右侧电机关闭角度
#define SERVO_BACK_GRAP_R_CLOSE_ANGLE 0
//左侧电机张开角度
#define SERVO_BACK_GRAP_R_OPEN_ANGLE (SERVO_BACK_GRAP_R_CLOSE_ANGLE + SERVO_BACK_GRAP_OPEN_ANGLE)


//后侧抬升舵机抬升角度
#define SERVO_BACK_LIFT_UP_ANGLE 20
//后侧抬升舵机定时器
#define SERVO_BACK_LIFT_TIM htim4
//后侧抬升舵机定时器通道
#define SERVO_BACK_LIFT_CHANNEL TIM_CHANNEL_4
//后侧抬升电机 下降
#define SERVO_BACK_LIFT_DOWN 180
//后侧抬升电机 上升
#define SERVO_BACK_LIFT_UP (SERVO_BACK_LIFT_DOWN - SERVO_BACK_LIFT_UP_ANGLE)

//定时器自动重装载寄存器数值
#define TIM1_ARR 100
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
