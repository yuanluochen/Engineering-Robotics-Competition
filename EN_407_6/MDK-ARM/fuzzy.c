#include "fuzzy_pid.h"
/************************************************************************************
* @fuction name：FUZZY_PID_CONTROL
* @fuction description： 模糊自适应控制算法，为了方便测试默认e、ec在[-3,3]区间，
* 如需改变e、ec范围，需引入量化因子(Ke、Kec=N/emax)、缩放因子(Ku=umax/N)。以下代码采
*用三角隶属函数求隶属度以及加权平均法解模糊，PID采用位置式PID算法，算法仅供参考，欢迎报错。
*************************************************************************************/
#include "usart.h"
#include "gpio.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

extern int real_code;
extern float my_speed_L;
extern float my_speed_R;

void motor_init()
{
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  //左电机
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL); 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);	
}

int Read_TIM3_Code(void)
{
	int Encoder_motor=0;
	Encoder_motor =  ((short)(TIM3 -> CNT));    //原来有 /4
	TIM3 -> CNT=0; //读编码器的计数值并计算出转速
	return Encoder_motor;
}

int Read_TIM2_Code(void)
{
	int Encoder_motor=0;
	Encoder_motor =  ((short)(TIM2 -> CNT));
	TIM2 -> CNT=0; //读编码器的计数值并计算出转速
	return Encoder_motor;
}

PID fuzzy(float e,float ec) // e 是目标值和反馈值的误差 ec是误差变化率(误差e的微分) 
{
     float etemp,ectemp;
     float eLefttemp,ecLefttemp;
     float eRighttemp ,ecRighttemp;

     int eLeftIndex,ecLeftIndex;
     int eRightIndex,ecRightIndex;
     PID      fuzzy_PID;
	
     etemp = e > 3.0 ? 0.0 : (e < - 3.0 ? 0.0 : (e >= 0.0 ? (e >= 2.0 ? 2.5: (e >= 1.0 ? 1.5 : 0.5)) : (e >= -1.0 ? -0.5 : (e >= -2.0 ? -1.5 : (e >= -3.0 ? -2.5 : 0.0) ))));

     eLeftIndex = (int)e;
     eRightIndex = eLeftIndex;
     eLeftIndex = (int)((etemp-0.5) + 3);        //[-3,3] -> [0,6]
     eRightIndex = (int)((etemp+0.5) + 3);

     eLefttemp =etemp == 0.0 ? 0.0:((etemp+0.5)-e);
     eRighttemp=etemp == 0.0 ? 0.0:( e-(etemp-0.5));

     ectemp = ec > 3.0 ? 0.0 : (ec < - 3.0 ? 0.0 : (ec >= 0.0 ? (ec >= 2.0 ? 2.5: (ec >= 1.0 ? 1.5 : 0.5)) : (ec >= -1.0 ? -0.5 : (ec >= -2.0 ? -1.5 : (ec >= -3.0 ? -2.5 : 0.0) ))));

     ecLeftIndex = (int)((ectemp-0.5) + 3);        //[-3,3] -> [0,6]
     ecRightIndex = (int)((ectemp+0.5) + 3);

     ecLefttemp =ectemp == 0.0 ? 0.0:((ectemp+0.5)-ec);
     ecRighttemp=ectemp == 0.0 ? 0.0:( ec-(ectemp-0.5));

/*************************************反模糊*************************************/
	fuzzy_PID.Kp = (eLefttemp * ecLefttemp *  fuzzyRuleKp[ecLeftIndex][eLeftIndex]
					+ eLefttemp * ecRighttemp * fuzzyRuleKp[ecRightIndex][eLeftIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKp[ecLeftIndex][eRightIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKp[ecRightIndex][eRightIndex]);

	fuzzy_PID.Ki =   (eLefttemp * ecLefttemp * fuzzyRuleKi[ecLeftIndex][eLeftIndex]
					+ eLefttemp * ecRighttemp * fuzzyRuleKi[ecRightIndex][eLeftIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKi[ecLeftIndex][eRightIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKi[ecRightIndex][eRightIndex]);

	fuzzy_PID.Kd = (eLefttemp * ecLefttemp *    fuzzyRuleKd[ecLeftIndex][eLeftIndex]
					+ eLefttemp * ecRighttemp * fuzzyRuleKd[ecRightIndex][eLeftIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKd[ecLeftIndex][eRightIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKd[ecRightIndex][eRightIndex]);
	return fuzzy_PID;

}


float FuzzyPid_Out(float tar,float cur)  // 目标值 , 实际值
{
   float e = 0,ec = 0;       // 误差e 误差变化率ec(误差e的微分)  系统死区设置量 不一定为零
	static PID pid= {4, 0, 0.01};
	static int sumE = 0;                   //累加偏差
	static int lastE = 0;
	PID OUT = {0, 0, 0};
	e = cur - tar;             //  实际值-目标值
	ec = e - lastE;            // 误差变化率
	sumE += e;
	lastE = ec;
	OUT = fuzzy(e, ec);      //模糊控制调整  kp，ki，kd

	return (pid.Kp+OUT.Kp)*e + (pid.Kd+OUT.Kd)*ec + (pid.Ki+OUT.Ki)*sumE; //最终输出值
}


float pwmout=0;
/* PID输出控制电机函数 */
void motor_Fuzzypid_control_R(float PID_OUT)
{
	pwmout=PID_OUT;
	if(PID_OUT>7100.0) PID_OUT=7100;
	if(PID_OUT<-7100.0) PID_OUT=-7100;

	if(fabs(PID_OUT)<=FuzzyPidTarge_Error)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		TIM1->CCR1=0;
//		printf("     stop \r\n");
	}	
	else if(PID_OUT-FuzzyPidTarge_Error>0)							
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		TIM1->CCR1=PID_OUT;
//		printf("     go \r\n");
	}
	else if(PID_OUT+FuzzyPidTarge_Error<0) 
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		TIM1->CCR1=-PID_OUT;
//		printf("     dowm \r\n");
	}
	
}

void motor_Fuzzypid_control_L(float PID_OUT)
{
	pwmout=PID_OUT;
	if(PID_OUT>7100.0) PID_OUT=7100;
	if(PID_OUT<-7100.0) PID_OUT=-7100;

	if(fabs(PID_OUT)<=FuzzyPidTarge_Error)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		TIM1->CCR2=0;
//		printf("     stop \r\n");
	}	
	else if(PID_OUT-FuzzyPidTarge_Error>0)							
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		TIM1->CCR2=PID_OUT;
//		printf("     go \r\n");
	}
	else if(PID_OUT+FuzzyPidTarge_Error<0) 
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		TIM1->CCR2=-PID_OUT;
//		printf("     dowm \r\n");
	}
	
}


void FuzzyPid_ControlPwm_L(float targe)
{
	motor_Fuzzypid_control_L(FuzzyPid_Out(targe,my_speed_L)+FuzzyPidMotor_OpenPwm);
}


void FuzzyPid_ControlPwm_R(float targe)
{
	motor_Fuzzypid_control_R(FuzzyPid_Out(targe,my_speed_R)+FuzzyPidMotor_OpenPwm);
}
