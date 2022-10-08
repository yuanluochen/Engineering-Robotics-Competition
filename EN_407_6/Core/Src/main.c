/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "mypwm.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stm32f4xx_it.h"
#include "fuzzy_pid.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int real_code_R=0;
float my_speed_R=0.0;
int real_code_L=0;
float my_speed_L=0.0;
int decode;
/* USER CODE END PTD */
extern vision_rxfifo_t vision_rxfifo;
 uint8_t rx_buffer_used[3]; //调用数据数组
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float tar_speed_L;
float tar_speed_R;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float use_contral;
int std_right_left;
int std_up_down;
int std_right_get;
int std_left_get;
int std_center_get;
int std_both_get;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t rx_len=0;  //接收到的数据长度
volatile uint8_t recv_end_flag=0;//接收成功标志位
uint8_t rx_buffer[50];//缓存数组

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	 vision_init();
	 
	motor_init();
	HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *)&htim6);  
  /* USER CODE END 2 */

  /* Infinite loop */
	Servomotor_Init();
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//该条件编译用于测试
#if 1
		for(int i = 0; i < 3; i++)
		{
			rx_buffer_used[i] = (RxBufVal())[i];
		}
#else
	
		rx_buffer_used[2] = 41;
	  
#endif
		if(rx_buffer_used[0] == 4)   //夹爪为前，直行
		{
			tar_speed_L = -MOTOR_SPEED_MID;
			tar_speed_R = -MOTOR_SPEED_MID;
		}else if(rx_buffer_used[0] == 2)  //夹爪为前，左转
		{
			tar_speed_L = -MOTOR_TURN_SPEED;
			tar_speed_R = MOTOR_TURN_SPEED;
		}
		else if(rx_buffer_used[0] == 3)   //夹爪为前，右转
		{
      //
			tar_speed_L = MOTOR_TURN_SPEED;
			tar_speed_R = -MOTOR_TURN_SPEED;
		}
		else if(rx_buffer_used[0] == 1)    //夹爪为前，后退
		{
			tar_speed_L = MOTOR_SPEED_MID;
			tar_speed_R = MOTOR_SPEED_MID;
		}
		else if(rx_buffer_used[0] == 0)
    {
			tar_speed_L = MOTOR_SPEED_STOP; //停止
			tar_speed_R = MOTOR_SPEED_STOP;
		}
	//右侧夹爪
	 if(rx_buffer_used[1] == 30) //down
    {
			  Set_SG90_angle(&htim4,TIM_CHANNEL_1,0,200,20); 
	}
		else if(rx_buffer_used[1]== 31)//up
    {
				Set_SG90_angle(&htim4,TIM_CHANNEL_1,90,200,20);
	}
	//前侧夹爪
		else if(rx_buffer_used[1]==20)//down
    {
				Set_SG90_angle(&htim4,TIM_CHANNEL_2,0,200,20);
	}
		else if(rx_buffer_used[1]==21)//up
    {
				Set_SG90_angle(&htim4,TIM_CHANNEL_2,90,200,20);
	}
	//左侧夹爪
		else if(rx_buffer_used[1]==30)//down
    {
				Set_SG90_angle(&htim4,TIM_CHANNEL_3,180,200,20);
		}		
		else if(rx_buffer_used[1]==31)//up
    {
				Set_SG90_angle(&htim4,TIM_CHANNEL_3,170,200,20);
		}
    else if (rx_buffer_used[2] == 40)//圆筒关闭
    {
        //关闭左右侧夹爪
        Set_SG90_angle(&SERVO_BACK_GRAP_TIM, SERVO_BACK_GRAP_L_CHANNEL, SERVO_BACK_GRAP_L_CLOSE_ANGLE, 200, 20); 
        Set_SG90_angle(&SERVO_BACK_GRAP_TIM, SERVO_BACK_GRAP_R_CHANNEL, SERVO_BACK_GRAP_R_CLOSE_ANGLE, 200, 20); 
    }
    else if (rx_buffer_used[2] == 41)//圆筒张开
    {
        //张开左右侧夹抓
        Set_SG90_angle(&SERVO_BACK_GRAP_TIM, SERVO_BACK_GRAP_L_CHANNEL, SERVO_BACK_GRAP_L_OPEN_ANGLE, 200, 20); 
        Set_SG90_angle(&SERVO_BACK_GRAP_TIM, SERVO_BACK_GRAP_R_CHANNEL, SERVO_BACK_GRAP_R_OPEN_ANGLE, 200, 20); 
    }
    else if (rx_buffer_used[2] == 50)//后侧抬升舵机下降
    {
        //圆筒下降
        Set_SG90_angle(&SERVO_BACK_LIFT_TIM, SERVO_BACK_LIFT_CHANNEL, SERVO_BACK_LIFT_DOWN, 200, 20);
    }
    else if (rx_buffer_used[2] == 51)//后侧抬升电机上升
    {
        //圆筒上升
        Set_SG90_angle(&SERVO_BACK_LIFT_TIM, SERVO_BACK_LIFT_CHANNEL, SERVO_BACK_LIFT_UP, 200, 20); 
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
