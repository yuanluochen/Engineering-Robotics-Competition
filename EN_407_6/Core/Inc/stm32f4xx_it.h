/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H
#include"main.h"
#ifdef __cplusplus
 extern "C" {
#endif

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
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void USART6_IRQHandler(void);
/* USER CODE BEGIN EFP */
typedef struct {
	fp32 yaw_fifo;//YAW信息
	fp32 pitch_fifo;//PITCH信息
	fp32 yaw_speed_fifo;//YAW速度信息
	fp32 yaw_disdance;//YAW速度信息
	int32_t pitch_speed_fifo;//PITCH速度信息
	bool_t rx_change_flag;//识别目标切换
	bool_t rx_flag;//识别到目标
	
	bool_t rx_update_flag;//视觉更新
} vision_rxfifo_t;

#define VISION_RX_LEN_2 58u
#define VISION_RX_LEN 29u

#define HEAD0_BASE 0
#define HEAD1_BASE 4
#define YAW_FIFO_BASE 8
#define PITCH_FIFO_BASE 12
#define YAW_SPEED_FIFO_BASE 16
#define PITCH_SPEED_FIFO_BASE 20
#define CHANGE_FLAG_FIFO_BASE 21

extern vision_rxfifo_t vision_rxfifo;
//extern uint8_t test_code[24];
extern uint8_t vision_rx_buf[2][VISION_RX_LEN_2];



extern void vision_init(void);
extern void vision_rx_decode(uint8_t *test_code);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

uint8_t* RxBufVal(void);

#endif /* __STM32F4xx_IT_H */
