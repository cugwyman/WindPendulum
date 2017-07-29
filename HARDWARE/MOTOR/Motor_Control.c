#include "Motor_Control.h"
#include "Motor_PWM.h"
#include "Motor_PID.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "uart6050.h"

/*------------------------------------------
 				ȫ�ֱ���				
------------------------------------------*/
M1TypeDef M1;
M2TypeDef M2;

float set_x = 0.0;                   //X�����õ�(M1)
float set_y = 0.0;                   //Y�����õ�(M2)
float R = 25.0; 					 //�뾶����(cm)
float Angle = 30.0;					 //�ڶ��Ƕ�����(��)
uint8_t RoundDir = 0; 				 //����ת����
///*------------------------------------------
// ��������:�����������λ
// ����˵��:ǿ�Ƹ�λ			
//------------------------------------------*/
//void MCU_Reset(void) 
//{  
//	__set_FAULTMASK(1);   // �ر������ж�
// 	NVIC_SystemReset();   // ��λ
//}
/*------------------------------------------
 ��������:��ʼ��M1�ṹ�����
 ����˵��:			
------------------------------------------*/
void M1TypeDef_Init(void)
{
	M1.CurPos    = 0.0;
	M1.PrevPos   = 0.0;
	M1.CurAcc    = 0.0;
	M1.PrevSpeed = 0.0;
 	M1.Offset    = 0.1;   //����ƫ����
	M1.CurSpeed  = 0.0;  //��ǰ�ٶ�ʸ��
	M1.PWM = 0;	         //PWM
}
/*------------------------------------------
 ��������:��ʼ��M2�ṹ�����
 ����˵��:			
------------------------------------------*/
void M2TypeDef_Init(void)
{
	M2.CurPos    = 0.0;
	M2.PrevPos   = 0.0;
	M2.CurAcc    = 0.0;
	M2.PrevSpeed = 0.0;
 	M2.Offset    = 0.1;   //����ƫ����
	M2.CurSpeed  = 0.0;  //��ǰ�ٶ�ʸ��
	M2.PWM = 0;	         //PWM		
}
/*------------------------------------------
 ��������:
------------------------------------------*/
void Mode_0(void)
{
	
		
}
/*------------------------------------------
 ��������:��1��PID����
 ����˵��:
------------------------------------------*/
void Mode_1(void)
{  
	const float priod = 1602.0;  //��������(����)
	static uint32_t MoveTimeCnt = 0;
	float A = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;			
	MoveTimeCnt += 5;							 //ÿ5ms����1��
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ��
	Omega = 2.0*3.14159*Normalization;			 //��2�н��й�һ������
	A = atan(((R+5)/82.0f))*57.2958f;				 //���ݰڷ�����Ƕ�A,82Ϊ�ڸ˾�����泤��cm
	set_y = A*sin(Omega); 
	
	PID_M1_SetPoint(0);	//X����PID����Ŀ��ֵsin
	PID_M1_SetKp(80);	
	PID_M1_SetKi(0);	 
	PID_M1_SetKd(2500);	
    
	PID_M2_SetPoint(set_y);		//Y����PID��λĿ��ֵ0
	PID_M2_SetKp(80);    
	PID_M2_SetKi(0);		
	PID_M2_SetKd(1500);
    
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos);	//Pitch
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos);

	if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
	if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;	
	
	if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
	if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;		
    MotorMove(M1.PWM,M2.PWM);
}
/*------------------------------------------
 ��������:��2��PID����
 ����˵��:
------------------------------------------*/
void Mode_2(void)
{
	const float priod = 1602.0;  //��������(����)
	static uint32_t MoveTimeCnt = 0;
	float A = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;				
	MoveTimeCnt += 5;							 //ÿ5ms����1��
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ��
	Omega = 2.0*3.14159*Normalization;			 //��2�н��й�һ������
	A = atan(((R+5)/82.0f))*57.2958f;//���ݰڷ�����Ƕ�A,82Ϊ�ڸ���ظ߶�
	set_x = A*sin(Omega);                        //�������ǰ�ڽ� 		
	
	PID_M1_SetPoint(set_x);	//X����PID����Ŀ��ֵsin
	PID_M1_SetKp(80);	
	PID_M1_SetKi(0);	 
	PID_M1_SetKd(2500);	
    
	PID_M2_SetPoint(0);		//Y����PID��λĿ��ֵ0
	PID_M2_SetKp(80);    
	PID_M2_SetKi(0);		
	PID_M2_SetKd(2500); 	 	
    
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos);	//X����PID����
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos);  //Y����PID����	
    
	if(M1.PWM > POWER_MAX) M1.PWM  =  POWER_MAX;//����޷�
	if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX; 	
    
	if(M2.PWM > POWER_MAX) M2.PWM  =  POWER_MAX;
	if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;	
	
	MotorMove(M1.PWM,M2.PWM);//������
}
/*------------------------------------------
 ��������:��3��PID����
 ����˵��:
------------------------------------------*/ 
void Mode_3(void)
{
	const float priod = 1602.0;  //��������(����)
	             //��λ���� 0, 10   20   30   40   50   60   70   80   90   100  110  120  130  140  150  160  170 180
	const float Phase[19]= {0,-0.1,-0.05,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.05,0.05,0.05,0.07,0};
	static uint32_t MoveTimeCnt = 0;
	float Ax = 0.0;
	float Ay = 0.0;
	float A = 0.0;
	uint32_t pOffset = 0;
	float Normalization = 0.0;
	float Omega = 0.0;
	pOffset = (uint32_t)(Angle/10.0f);			 //��λ���������±�
	MoveTimeCnt += 5;							 //ÿ5ms����1��
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ��
	Omega = 2.0*3.14159*Normalization;			 //��2�н��й�һ������
	A = atan(((R+5)/82.0f))*57.2958f;//���ݰڷ�����Ƕ�A,82Ϊ�ڸ���ظ߶�                   						
	Ax = A*cos(Angle*0.017453);	 //�����X����ڷ�����0.017453Ϊ����ת��
	Ay = A*sin(Angle*0.017453);	 //�����Y����ڷ�����
	set_x = Ax*sin(Omega); 		 //�����X����ǰ�ڽ�
	set_y = Ay*sin(Omega+Phase[pOffset]); //�����Y����ǰ�ڽ�
		
	PID_M1_SetPoint(set_x);	//X����PID����Ŀ��ֵsin
	PID_M1_SetKp(80);	
	PID_M1_SetKi(0);	 
	PID_M1_SetKd(2500);	
    
	PID_M2_SetPoint(0);		//Y����PID��λĿ��ֵ0
	PID_M2_SetKp(80);    
	PID_M2_SetKi(0);		
	PID_M2_SetKd(2500); 	 
	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos);	//Pitch
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos);  //Roll
	
	if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
	if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;
			 	
	if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
	if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;		
	MotorMove(M1.PWM,M2.PWM);	
	
}
/*------------------------------------------
 ��������:��4��PID����
 ����˵��:
------------------------------------------*/ 
void Mode_4(void)
{	
	if(fabs(M1.CurPos) < 45.0 && fabs(M2.CurPos) < 45.0)	//С��45�ȲŽ����ƶ�
	{		
		PID_M1_SetPoint(0);	  //X����PID��λĿ��ֵ0
		PID_M1_SetKp(20); 		
		PID_M1_SetKi(0);     
		PID_M1_SetKd(4000);

		PID_M2_SetPoint(0);	  //Y����PID��λĿ��ֵ0
		PID_M2_SetKp(25);  		
		PID_M2_SetKi(0);    
		PID_M2_SetKd(4000);
			
		M1.PWM = PID_M1_PosLocCalc(M1.CurPos); //Pitch
		M2.PWM = PID_M2_PosLocCalc(M2.CurPos); //Roll
		
		if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
		if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;

		if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
		if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;
	}
	else	
	{
	 	M1.PWM = 0;
		M2.PWM = 0;	
	}
	
	MotorMove(M1.PWM,M2.PWM);
}
/*------------------------------------------
 ��������:��5��PID����
 ����˵��:
------------------------------------------*/
void Mode_5(void)
{
	const float priod = 1602.0;  //��������(����)
	static uint32_t MoveTimeCnt = 0;
	float Ax = 0.0,Ay=0.0;
	float phase = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;
	
	MoveTimeCnt += 5;							 //ÿ5ms����1��
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ��
	Omega = 2.0*3.14159*Normalization;			 //��2�н��й�һ������				
	Ax = atan(((R+5)/82.0f))*57.2958f;    //���ݰ뾶�����Ӧ�����A
	Ay= atan(((R+5)/82.0f))*57.2958f;
	
	if(RoundDir == 0)       	  
		phase = 3.141592/2.0;		 //��ʱ����ת��λ��90�� 
	else if(RoundDir == 1)  
		phase = (3.0*3.141592)/2.0;	 //˳ʱ����ת��λ��270��
	
	set_x = Ax*sin(Omega);			 //�����X����ǰ�ڽ�
	set_y = Ay*sin(Omega+phase); 	 //�����Y����ǰ�ڽ�
	 
	PID_M1_SetPoint(set_x);	//X����PID����Ŀ��ֵsin
	PID_M1_SetKp(80);	//24
	PID_M1_SetKi(0);	 //0.1
	PID_M1_SetKd(2500);//3000

	PID_M2_SetPoint(set_y);	//Y����PID����Ŀ��ֵsin
	PID_M2_SetKp(50);    
	PID_M2_SetKi(0);		
	PID_M2_SetKd(1500); 		 
	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos); //Pitch
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos); //Roll
	
	if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
	if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;
			 	
	if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
	if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;		

	MotorMove(M1.PWM,M2.PWM);
	
}
/*------------------------------------------
 ��������:��6��PID����
 ����˵��:
------------------------------------------*/
void Mode_6(void)
{

}
/*------------------------------------------
 ��������:����ײ���������
 ����˵��:
------------------------------------------*/
void MotorMove(int32_t pwm1,int32_t pwm2)
{
	if(pwm1 >= 0)
	{
		PWM_M3_Backward(pwm1+20);	
		PWM_M2_Forward(0);
	}
	else if(pwm1 < 0)
	{
	 	PWM_M3_Forward(abs(0));
		PWM_M2_Backward(abs(pwm1+20));
	}

	if(pwm2 >= 0)
	{
		PWM_M1_Forward(0);
	 	PWM_M4_Backward(pwm2-30);
	}
	else if(pwm2 < 0)
	{	
		PWM_M1_Backward(abs(pwm2+30));
	 	PWM_M4_Forward(abs(0));
	} 	
}


