#include "fuzzy_pid.h"
/************************************************************************************
* @fuction name��FUZZY_PID_CONTROL
* @fuction description�� ģ������Ӧ�����㷨��Ϊ�˷������Ĭ��e��ec��[-3,3]���䣬
* ����ı�e��ec��Χ����������������(Ke��Kec=N/emax)����������(Ku=umax/N)�����´����
*�����������������������Լ���Ȩƽ������ģ����PID����λ��ʽPID�㷨���㷨�����ο�����ӭ����
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
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  //����
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL); 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);	
}

int Read_TIM3_Code(void)
{
	int Encoder_motor=0;
	Encoder_motor =  ((short)(TIM3 -> CNT));    //ԭ���� /4
	TIM3 -> CNT=0; //���������ļ���ֵ�������ת��
	return Encoder_motor;
}

int Read_TIM2_Code(void)
{
	int Encoder_motor=0;
	Encoder_motor =  ((short)(TIM2 -> CNT));
	TIM2 -> CNT=0; //���������ļ���ֵ�������ת��
	return Encoder_motor;
}

PID fuzzy(float e,float ec) // e ��Ŀ��ֵ�ͷ���ֵ����� ec�����仯��(���e��΢��) 
{
     float etemp,ectemp;
     float eLefttemp,ecLefttemp;
     float eRighttemp ,ecRighttemp;

     int eLeftIndex,ecLeftIndex;
     int eRightIndex,ecRightIndex;
     PID      fuzzy_PID;
	
     etemp = e > 3.0f ? 0.0f : (e < - 3.0f ? 0.0f : (e >= 0.0f ? (e >= 2.0f ? 2.5f: (e >= 1.0f ? 1.5f : 0.5f)) : (e >= -1.0f ? -0.5f : (e >= -2.0f ? -1.5f : (e >= -3.0f ? -2.5f : 0.0f) ))));

     eLeftIndex = (int)e;
     eRightIndex = eLeftIndex;
     eLeftIndex = (int)((etemp-0.5f) + 3);        //[-3,3] -> [0,6]
     eRightIndex = (int)((etemp+0.5f) + 3);

     eLefttemp =etemp == 0.0f ? 0.0f:((etemp+0.5f)-e);
     eRighttemp=etemp == 0.0f ? 0.0f:( e - (etemp - 0.5f));

     ectemp = ec > 3.0f ? 0.0f : (ec < - 3.0f ? 0.0f : (ec >= 0.0f ? (ec >= 2.0f ? 2.5f: (ec >= 1.0f ? 1.5f : 0.5f)) : (ec >= -1.0f ? -0.5f : (ec >= -2.0f ? -1.5f : (ec >= -3.0f ? -2.5f : 0.0f) ))));

     ecLeftIndex = (int)((ectemp-0.5f) + 3);        //[-3,3] -> [0,6]
     ecRightIndex = (int)((ectemp+0.5f) + 3);

     ecLefttemp =ectemp == 0.0f ? 0.0f:((ectemp+0.5f)-ec);
     ecRighttemp=ectemp == 0.0f ? 0.0f:( ec-(ectemp-0.5f));

/*************************************��ģ��*************************************/
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


float FuzzyPid_Out(float tar,float cur)  // Ŀ��ֵ , ʵ��ֵ
{
   float e = 0,ec = 0;       // ���e ���仯��ec(���e��΢��)  ϵͳ���������� ��һ��Ϊ��
	static PID pid= {15, 0, 0.01};
	static int sumE = 0;                   //�ۼ�ƫ��
	static int lastE = 0;
	PID OUT = {0, 0, 0};
	e = cur - tar;             //  ʵ��ֵ-Ŀ��ֵ
	ec = e - lastE;            // ���仯��
	sumE += e;
	lastE = ec;
	OUT = fuzzy(e, ec);      //ģ�����Ƶ���  kp��ki��kd

	return (pid.Kp+OUT.Kp)*e + (pid.Kd+OUT.Kd)*ec + (pid.Ki+OUT.Ki)*sumE; //�������ֵ
}


float pwmout=0;
/* PID������Ƶ������ */
void motor_Fuzzypid_control_R(float PID_OUT)
{
	pwmout=PID_OUT;
	if(PID_OUT>7100.0f) PID_OUT=7100;
	if(PID_OUT<-7100.0f) PID_OUT=-7100;

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
	if(PID_OUT>7100.0f) PID_OUT=7100;
	if(PID_OUT<-7100.0f) PID_OUT=-7100;
#if 1
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
#else
		if(fabs(PID_OUT)<=FuzzyPidTarge_Error)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		//TIM1->CCR2=0;
//		printf("     stop \r\n");
	}	
	else if(PID_OUT-FuzzyPidTarge_Error>0)							
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		//TIM1->CCR2=PID_OUT;

//		printf("     go \r\n");
	}
	else if(PID_OUT+FuzzyPidTarge_Error<0) 
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		//TIM1->CCR2=-PID_OUT;

//		printf("     dowm \r\n");
	}
#endif
	
}


void FuzzyPid_ControlPwm_L(float targe)
{
	motor_Fuzzypid_control_L(FuzzyPid_Out(targe,my_speed_L)+FuzzyPidMotor_OpenPwm);
}


void FuzzyPid_ControlPwm_R(float targe)
{
	motor_Fuzzypid_control_R(FuzzyPid_Out(targe,my_speed_R)+FuzzyPidMotor_OpenPwm);
}
