#ifndef __MOTOR__PID__H__
#define __MOTOR__PID__H__

#include "stm32f10x.h"


/*------------------------------------------
 				PID结构体				
------------------------------------------*/
typedef struct
{
	float  SetPoint; 	//  设定目标 Desired Value 
	double  SumError;		//	误差累计 
		
	float  Proportion;      //  比例常数 Proportional Const 
	float  Integral;        //  积分常数 Integral Const
	float  Derivative;      //  微分常数 Derivative Const

	float LastError;     //  Error[-1]
	float PrevError;     //  Error[-2]

}PIDTypdDef;

extern PIDTypdDef M1PID;
extern PIDTypdDef M2PID;

int32_t  PID_M1_PosLocCalc(float NextPoint);

void     PID_M1_Init(void);
void 	 PID_M1_SetKd(float dKdd);
void 	 PID_M1_SetKi(float dKii);
void 	 PID_M1_SetKp(float dKpp);
void 	 PID_M1_SetPoint(float setpoint);

int32_t  PID_M2_PosLocCalc(float NextPoint);
void     PID_M2_Init(void);
void 	 PID_M2_SetKd(float dKdd);
void 	 PID_M2_SetKi(float dKii);
void 	 PID_M2_SetKp(float dKpp);
void 	 PID_M2_SetPoint(float setpoint);

#endif

