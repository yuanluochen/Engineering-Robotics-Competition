#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H

#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3

//注意：static PID pid= {0, 0, 0};  需要自己赋值

static const float fuzzyRuleKp[7][7]={
	PL,	PL,	PM,	PM,	PS,	PS,	ZE,
	PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
	PM,	PM,	PM,	PS,	ZE,	NS,	NM,
	PM,	PS,	PS,	ZE,	NS,	NM,	NM,
	PS,	PS,	ZE,	NS,	NS,	NM,	NM,
	ZE,	ZE,	NS,	NM,	NM,	NM,	NL,
	ZE,	NS,	NS,	NM,	NM,	NL,	NL
};

static const float fuzzyRuleKi[7][7]={
	NL,	NL,	NL,	NM,	NM,	ZE,	ZE,
	NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
	NM,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	ZE,	PS,	PS,	PM,
	NS,	NS,	ZE,	PS,	PS,	PM,	PM,
	ZE,	ZE,	PS,	PM,	PM,	PL,	PL,
	ZE,	ZE,	PS,	PM,	PL,	PL,	PL
};

static const float fuzzyRuleKd[7][7]={
	PS,	PS,	ZE,	ZE,	ZE,	PL,	PL,
	NS,	NS,	NS,	NS,	ZE,	NS,	PM,
	NL,	NL,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	NS,	ZE,	PS,	PS,
	PS,	ZE,	ZE,	ZE,	ZE,	PL,	PL
};

typedef struct{
	float Kp;
	float Ki;
	float Kd;
}PID;

#define MOTOR_TIM htim1
#define MOTOR_LEFT_CHANNEL TIM_CHANNEL_1
#define MOTOR_RIGHT_CHANNEL TIM_CHANNEL_2

#define FuzzyPidMotor_OpenPwm 3500 // 电机死区
#define FuzzyPidTarge_Error 0 // 目标值允许误差

int Read_TIM3_Code(void);
int Read_TIM2_Code(void);

PID fuzzy(float e,float ec); // 模糊PID自整定函数
float FuzzyPid_Out(float tar,float cur);  // 目标值 , 实际值
void motor_Fuzzypid_control_L(float PID_OUT);
void motor_Fuzzypid_control_R(float PID_OUT);
void motor_init(void);   //初始化

void FuzzyPid_ControlPwm_L(float targe);
void FuzzyPid_ControlPwm_R(float targe);

extern float pwmout;

#endif
