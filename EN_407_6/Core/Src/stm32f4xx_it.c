/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fuzzy_pid.h"
#include "mypwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern int leftSpeedNow ;
extern int rightSpeedNow;
extern float tar_speed_R;
extern float tar_speed_L;

extern volatile uint8_t rx_len;//接收到的数据长度
extern volatile uint8_t recv_end_flag; //接收完成标志位
extern uint8_t rx_buffer[50]; //数据缓存数组
extern float use_contral;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */
extern int real_code_R;
extern float my_speed_R;
extern int real_code_L;
extern float my_speed_L;
static uint8_t rx_buffer_used[3]; //调用数据数组
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
extern int decode;
float Date_Decoder(uint8_t* rx_buffer)  //处理数据
{
    int rx_use=(rx_buffer[0] << 3 * 8) | (rx_buffer[1] << 2 * 8) | (rx_buffer[2] << 1 * 8) | (rx_buffer[3] << 0 * 8);
    // rx_use = (float) rx_use / 10000.0;
    // return rx_use;
    return (float)(rx_use / 10000.0);
}


uint8_t vision_rx_buf[2][VISION_RX_LEN_2];

vision_rxfifo_t vision_rxfifo = {0};

void vision_init(void)
{
	    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(vision_rx_buf[0]);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(vision_rx_buf[1]);
    //data length
    //数据长度
    hdma_usart6_rx.Instance->NDTR = 48u;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

}


void vision_rx_decode(uint8_t *test_code)
{
	if((fp32)((test_code[HEAD0_BASE+0] << 8*3) | (test_code[HEAD0_BASE+1] << 8*2)
					| (test_code[HEAD0_BASE+2] << 8*1) | (test_code[HEAD0_BASE+3] << 8*0)) == 0x34)
	{
		if((fp32)((test_code[HEAD1_BASE+0] << 8*3) | (test_code[HEAD1_BASE+1] << 8*2)
						| (test_code[HEAD1_BASE+2] << 8*1) | (test_code[HEAD1_BASE+3] << 8*0)) == 0x43)
		{
			vision_rxfifo.rx_flag = 1;
			
			vision_rxfifo.yaw_fifo 	 = (test_code[YAW_FIFO_BASE+0] << 8*3) | (test_code[YAW_FIFO_BASE+1] << 8*2)
															 | (test_code[YAW_FIFO_BASE+2] << 8*1) | (test_code[YAW_FIFO_BASE+3] << 8*0);
			vision_rxfifo.pitch_fifo = (test_code[PITCH_FIFO_BASE+0] << 8*3) | (test_code[PITCH_FIFO_BASE+1] << 8*2)
															 | (test_code[PITCH_FIFO_BASE+2] << 8*1) | (test_code[PITCH_FIFO_BASE+3] << 8*0);
			vision_rxfifo.yaw_disdance  = (test_code[YAW_SPEED_FIFO_BASE+0] << 8*3) | (test_code[YAW_SPEED_FIFO_BASE+1] << 8*2)
																		| (test_code[YAW_SPEED_FIFO_BASE+2] << 8*1) | (test_code[YAW_SPEED_FIFO_BASE+3] << 8*0);
			vision_rxfifo.pitch_speed_fifo  = (test_code[PITCH_SPEED_FIFO_BASE+0] << 8*3) | (test_code[PITCH_SPEED_FIFO_BASE+1] << 8*2)
																			| (test_code[PITCH_SPEED_FIFO_BASE+2] << 8*1) | (test_code[PITCH_SPEED_FIFO_BASE+3] << 8*0);
			vision_rxfifo.rx_change_flag = test_code[CHANGE_FLAG_FIFO_BASE+3];
			
			vision_rxfifo.rx_update_flag = 1;
			///
			vision_rxfifo.yaw_fifo 		= (fp32)vision_rxfifo.yaw_fifo 	 / 10000.0f;
			vision_rxfifo.pitch_fifo  = (fp32)vision_rxfifo.pitch_fifo / 10000.0f;      //angle
			vision_rxfifo.yaw_disdance=(fp32)vision_rxfifo.yaw_disdance/10000.0f;       //p
		}
	}
	else if((fp32)((test_code[HEAD0_BASE+0] << 8*3) | (test_code[HEAD0_BASE+1] << 8*2)
					| (test_code[HEAD0_BASE+2] << 8*1) | (test_code[HEAD0_BASE+3] << 8*0)) == 0x66)
	{
		if((fp32)((test_code[HEAD1_BASE+0] << 8*3) | (test_code[HEAD1_BASE+1] << 8*2)
						| (test_code[HEAD1_BASE+2] << 8*1) | (test_code[HEAD1_BASE+3] << 8*0)) == 0x66)
		{
			vision_rxfifo.rx_flag 	= 0;
			
			vision_rxfifo.yaw_fifo 	= 180;
			vision_rxfifo.pitch_fifo = 180;
			vision_rxfifo.yaw_disdance = 0;
			vision_rxfifo.pitch_speed_fifo  = 0;
			vision_rxfifo.rx_change_flag	= 0;
			
			vision_rxfifo.rx_update_flag = 1;
		}
	}

}
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if(huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&huart6);

	}
	else if(USART6->SR & UART_FLAG_IDLE)//空闲中断
	{
		volatile static uint16_t this_time_rx_len = 0;//存放接收到的数据长度

		__HAL_UART_CLEAR_PEFLAG(&huart6);

		if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */

			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = VISION_RX_LEN_2 - hdma_usart6_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart6_rx.Instance->NDTR = VISION_RX_LEN_2;

			//t memory buffer 1
			//设定缓冲区1
			hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
			
			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			//if(this_time_rx_len == VISION_RX_LEN)
			{
				 //数据接收，解码
				vision_rx_decode(vision_rx_buf[0]);
			}
		}else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = VISION_RX_LEN_2 - hdma_usart6_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart6_rx.Instance->NDTR = VISION_RX_LEN_2;

			//t memory buffer 0
			//设定缓冲区0
			DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
			
			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			//if(this_time_rx_len == VISION_RX_LEN)
			{
				 ////数据接收，解码
				vision_rx_decode(vision_rx_buf[1]);
			}
		}
	}
  /* USER CODE END USART1_IRQn 0 */
			rx_buffer_used[0]=vision_rxfifo.yaw_fifo ;
			rx_buffer_used[1]=vision_rxfifo.pitch_fifo ;      //angle
			rx_buffer_used[2]=vision_rxfifo.yaw_disdance;       //p
//			if(vision_rxfifo.yaw_fifo==4)   //圆筒为前，直行
//		{
//			tar_speed_L=5;
//			tar_speed_R=5;
//		}else if(vision_rxfifo.yaw_fifo==5)  //圆筒为前，左转
//		{
//			tar_speed_L=5;
//			tar_speed_R=6;
//		}
//		else if(vision_rxfifo.yaw_fifo==6)   //圆筒为前，右转
//		{
//			tar_speed_L=5;
//			tar_speed_R=6;
//		}
//		else if(vision_rxfifo.yaw_fifo==1)    //夹子为前，直行
//		{
//			tar_speed_L=-5;
//			tar_speed_R=-5;
//		}		
//		else if(vision_rxfifo.yaw_fifo==2)    //夹子为前，左转
//		{
//			tar_speed_L=-5;
//			tar_speed_R=-6;
//		}
//		else if(vision_rxfifo.yaw_fifo==3)    //夹子为前，右转
//		{
//			tar_speed_L=-6;
//			tar_speed_R=-5;
//		}
//		else {
//			tar_speed_L=0;                        //停止
//			tar_speed_R=0;
//		}
//	 if(vision_rxfifo.pitch_fifo==10){
//			Set_SG90_angle(&htim4,TIM_CHANNEL_1,0,200,20);      //左夹子
//		}
//		else if(vision_rxfifo.pitch_fifo==11){
//				Set_SG90_angle(&htim4,TIM_CHANNEL_1,90,200,20);
//		}
//		else if(vision_rxfifo.pitch_fifo==20){
//				Set_SG90_angle(&htim4,TIM_CHANNEL_2,0,200,20);
//		}
//		else if(vision_rxfifo.pitch_fifo==21){
//				Set_SG90_angle(&htim4,TIM_CHANNEL_2,90,200,20);
//		}
//		else if(vision_rxfifo.pitch_fifo==30){
//				Set_SG90_angle(&htim4,TIM_CHANNEL_3,180,200,20);
//		}		
//		else if(vision_rxfifo.pitch_fifo==31){
//				Set_SG90_angle(&htim4,TIM_CHANNEL_3,170,200,20);
//		}
//		
	HAL_TIM_PeriodElapsedCallback(&htim6);
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART1_IRQn 1 */
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
int old_speed_L=0;
int old_speed_R=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIM6)
	{
			real_code_R=Read_TIM3_Code();
			real_code_L=Read_TIM2_Code();

			my_speed_L=real_code_L*0.8+old_speed_L*0.2;
			old_speed_L=real_code_L;
	
			my_speed_R=real_code_R*0.8+old_speed_R*0.2;
			old_speed_R=real_code_R;
			
		motor_Fuzzypid_control_L( FuzzyPid_Out(tar_speed_L,my_speed_L));
		motor_Fuzzypid_control_R( FuzzyPid_Out(tar_speed_R,my_speed_R));
	}
}


uint8_t* RxBufVal(void)
{
  return rx_buffer_used;
}
/* USER CODE END 1 */
