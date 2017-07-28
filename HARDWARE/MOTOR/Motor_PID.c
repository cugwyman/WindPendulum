#include "motor_control.h"
#include "Motor_PWM.h"
#include "Motor_PID.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
/*------------------------------------------
 				声明变量				
------------------------------------------*/
PIDTypdDef M1PID;
PIDTypdDef M2PID;
/*------------------------------------------
 函数功能:初始化M1PID结构体参数
 函数说明:			
------------------------------------------*/
void PID_M1_Init(void)
{
    M1PID.LastError  = 0;			//Error[-1]
    M1PID.PrevError  = 0;			//Error[-2]
	M1PID.Proportion = 0;			//比例常数 Proportional Const
    M1PID.Integral   = 0;			//积分常数 Integral Const
    M1PID.Derivative = 0;			//微分常数 Derivative Const
    M1PID.SetPoint   = 0;
	M1PID.SumError   = 0;
}
/*------------------------------------------
 函数功能:初始化M2PID结构体参数
 函数说明:			
------------------------------------------*/
void PID_M2_Init(void)
{
    M2PID.LastError  = 0;			//Error[-1]
    M2PID.PrevError  = 0;			//Error[-2]
   	M2PID.Proportion = 0;			//比例常数 Proportional Const
    M2PID.Integral   = 0;			//积分常数 Integral Const
    M2PID.Derivative = 0;			//微分常数 Derivative Const
    M2PID.SetPoint   = 0;
	M2PID.SumError   = 0;
}
/*------------------------------------------
 函数功能:设置M1PID期望值
 函数说明:			
------------------------------------------*/
void PID_M1_SetPoint(float setpoint)
{	
	M1PID.SetPoint = setpoint;	
}
/*------------------------------------------
 函数功能:设置M2期望值
 函数说明:			
------------------------------------------*/
void PID_M2_SetPoint(float setpoint)
{	
	M2PID.SetPoint = setpoint;	
}
/*------------------------------------------
 函数功能:设置M1PID比例系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1_SetKp(float dKpp)
{	
	M1PID.Proportion = dKpp;	
}
/*------------------------------------------
 函数功能:设置M2比例系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M2_SetKp(float dKpp)
{	
	M2PID.Proportion = dKpp;	
}
/*------------------------------------------
 函数功能:设置M1PID积分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1_SetKi(float dKii)
{	
	M1PID.Integral = dKii;	
}
/*------------------------------------------
 函数功能:设置M2积分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M2_SetKi(float dKii)
{	
	M2PID.Integral = dKii;	
}
/*------------------------------------------
 函数功能:设置M1PID微分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1_SetKd(float dKdd)
{	
	M1PID.Derivative = dKdd;
}
/*------------------------------------------
 函数功能:设置M2微分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M2_SetKd(float dKdd)
{	
	M2PID.Derivative = dKdd;
}
/*------------------------------------------
 函数功能:电机1位置式PID计算
 函数说明:		
------------------------------------------*/
int32_t PID_M1_PosLocCalc(float NextPoint)
{
    register float  iError,dError;

	iError = M1PID.SetPoint - NextPoint;        // 偏差
	M1PID.SumError += iError;				    // 积分
	if(M1PID.SumError > 1000.0)					//积分限幅2300
		M1PID.SumError = 1000;
	else if(M1PID.SumError < -1000.0)
		M1PID.SumError = -1000;	
	dError = iError - M1PID.LastError; 			// 当前微分
	M1PID.LastError = iError;
	
	return(int32_t)(  M1PID.Proportion * iError           	// 比例项
          		    + M1PID.Integral   * M1PID.SumError 		// 积分项
          		    + M1PID.Derivative * dError);
}

/*------------------------------------------
 函数功能:电机2位置式PID计算
 函数说明:			
------------------------------------------*/
int32_t PID_M2_PosLocCalc(float NextPoint)
{
	register float  iError,dError;

	iError = M2PID.SetPoint - NextPoint;        // 偏差
	M2PID.SumError += iError;
	if(M2PID.SumError > 1000.0)					//积分限幅
		M2PID.SumError = 1000;
	else if(M2PID.SumError < -1000.0)
		M2PID.SumError = -1000;
	dError = iError - M2PID.LastError; 			// 当前微分
	M2PID.LastError = iError;
	
	return(int32_t)(  M2PID.Proportion * iError           	// 比例项
          		    + M2PID.Integral   * M2PID.SumError 		// 积分项
          		    + M2PID.Derivative * dError);
}
