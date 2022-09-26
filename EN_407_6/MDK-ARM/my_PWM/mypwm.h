#ifndef __MYTPWM_H
#define __MYTPWM_H
#include "mypwm.h"
/* USER CODE BEGIN 0 */
#include "stm32f4xx_hal.h"
#include "tim.h"

//抬升夹具,与夹子朝向一致判断左右
#define left_up 2     //正对左范围0-40°  夹紧度数为13°   初始为0
#define right_up 1    //正对右范围180-140°  夹紧度数为167°  初始为180°
#define up_down 4
#define Lifting_Clamp htim4
#define Opening_Closing htim5

//抓取夹具,与夹子朝向一致判断左右
#define left_get 1 
#define right_get 3
#define center_get 2
#define Move_Thing_Clamp htim4


	
void Servomotor_Init(void);
void Set_SG90_angle(TIM_HandleTypeDef * htim,uint32_t Channel,uint8_t angle,uint32_t countPeriod,uint32_t CycleTime);


#endif
