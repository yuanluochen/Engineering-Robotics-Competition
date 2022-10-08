#include "mypwm.h"


#include "stm32f4xx_hal.h"
#include "tim.h"

// angle:�Ƕ�ֵ��0~180

void Servomotor_Init()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	Set_SG90_angle(&htim4,TIM_CHANNEL_1, 0,200,20);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	Set_SG90_angle(&htim4,TIM_CHANNEL_2,0,200,20);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	Set_SG90_angle(&htim4,TIM_CHANNEL_3,180,200,20);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	Set_SG90_angle(&htim4,TIM_CHANNEL_4, SERVO_BACK_LIFT_DOWN,200,20);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	Set_SG90_angle(&htim5,TIM_CHANNEL_1,180,200,20);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	Set_SG90_angle(&htim5,TIM_CHANNEL_2,0,200,20);
}



/*
* htim:Ҫ���õĶ�ʱ�����
* Channel��PWM���ͨ��
* countPeriod��PWMһ�����ڼ���ֵ������Ϊ200
* CycleTime��һ�����ڶ���ms������Ϊ20
*
*/
void Set_SG90_angle(TIM_HandleTypeDef * htim,uint32_t Channel,uint8_t angle,uint32_t countPeriod,uint32_t CycleTime)
{
	uint16_t compare_value=0;
	if(angle<=180)
	{
		compare_value=0.5*countPeriod/CycleTime+angle*countPeriod/CycleTime/90;
		__HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
	}
}


/* USER CODE END 0 */
