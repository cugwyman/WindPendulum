#include "motor_control.h"
#include "Motor_PWM.h"
#include "Motor_PID.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
/*------------------------------------------
 				��������				
------------------------------------------*/
PIDTypdDef M1PID;
PIDTypdDef M2PID;
/*------------------------------------------
 ��������:��ʼ��M1PID�ṹ�����
 ����˵��:			
------------------------------------------*/
void PID_M1_Init(void)
{
    M1PID.LastError  = 0;			//Error[-1]
    M1PID.PrevError  = 0;			//Error[-2]
	M1PID.Proportion = 0;			//�������� Proportional Const
    M1PID.Integral   = 0;			//���ֳ��� Integral Const
    M1PID.Derivative = 0;			//΢�ֳ��� Derivative Const
    M1PID.SetPoint   = 0;
	M1PID.SumError   = 0;
}
/*------------------------------------------
 ��������:��ʼ��M2PID�ṹ�����
 ����˵��:			
------------------------------------------*/
void PID_M2_Init(void)
{
    M2PID.LastError  = 0;			//Error[-1]
    M2PID.PrevError  = 0;			//Error[-2]
   	M2PID.Proportion = 0;			//�������� Proportional Const
    M2PID.Integral   = 0;			//���ֳ��� Integral Const
    M2PID.Derivative = 0;			//΢�ֳ��� Derivative Const
    M2PID.SetPoint   = 0;
	M2PID.SumError   = 0;
}
/*------------------------------------------
 ��������:����M1PID����ֵ
 ����˵��:			
------------------------------------------*/
void PID_M1_SetPoint(float setpoint)
{	
	M1PID.SetPoint = setpoint;	
}
/*------------------------------------------
 ��������:����M2����ֵ
 ����˵��:			
------------------------------------------*/
void PID_M2_SetPoint(float setpoint)
{	
	M2PID.SetPoint = setpoint;	
}
/*------------------------------------------
 ��������:����M1PID����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1_SetKp(float dKpp)
{	
	M1PID.Proportion = dKpp;	
}
/*------------------------------------------
 ��������:����M2����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M2_SetKp(float dKpp)
{	
	M2PID.Proportion = dKpp;	
}
/*------------------------------------------
 ��������:����M1PID����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1_SetKi(float dKii)
{	
	M1PID.Integral = dKii;	
}
/*------------------------------------------
 ��������:����M2����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M2_SetKi(float dKii)
{	
	M2PID.Integral = dKii;	
}
/*------------------------------------------
 ��������:����M1PID΢��ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1_SetKd(float dKdd)
{	
	M1PID.Derivative = dKdd;
}
/*------------------------------------------
 ��������:����M2΢��ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M2_SetKd(float dKdd)
{	
	M2PID.Derivative = dKdd;
}
/*------------------------------------------
 ��������:���1λ��ʽPID����
 ����˵��:		
------------------------------------------*/
int32_t PID_M1_PosLocCalc(float NextPoint)
{
    register float  iError,dError;

	iError = M1PID.SetPoint - NextPoint;        // ƫ��
	M1PID.SumError += iError;				    // ����
	if(M1PID.SumError > 1000.0)					//�����޷�2300
		M1PID.SumError = 1000;
	else if(M1PID.SumError < -1000.0)
		M1PID.SumError = -1000;	
	dError = iError - M1PID.LastError; 			// ��ǰ΢��
	M1PID.LastError = iError;
	
	return(int32_t)(  M1PID.Proportion * iError           	// ������
          		    + M1PID.Integral   * M1PID.SumError 		// ������
          		    + M1PID.Derivative * dError);
}

/*------------------------------------------
 ��������:���2λ��ʽPID����
 ����˵��:			
------------------------------------------*/
int32_t PID_M2_PosLocCalc(float NextPoint)
{
	register float  iError,dError;

	iError = M2PID.SetPoint - NextPoint;        // ƫ��
	M2PID.SumError += iError;
	if(M2PID.SumError > 1000.0)					//�����޷�
		M2PID.SumError = 1000;
	else if(M2PID.SumError < -1000.0)
		M2PID.SumError = -1000;
	dError = iError - M2PID.LastError; 			// ��ǰ΢��
	M2PID.LastError = iError;
	
	return(int32_t)(  M2PID.Proportion * iError           	// ������
          		    + M2PID.Integral   * M2PID.SumError 		// ������
          		    + M2PID.Derivative * dError);
}
