#include "Motor_PWM.h"
#include "Motor_Control.h"
#include "stdio.h"
/*------------------------------------------
 ��������:����TIM4�������PWMʱ�õ���I/O
 ����˵��:PB6 - TIM4_CH1 - M4_PWM
	   	    PB7 - TIM4_CH2 - M1_PWM
	   	    PB8 - TIM4_CH3 - M2_PWM
		      PB9 - TIM4_CH4 - M3_PWM
		  
		  - M1_DIR  -> PD14
 		  - M1_STOP -> PD12

		  - M2_DIR  -> PD15
 		  - M2_STOP -> PD13
		  
		  - M3_DIR  -> PD10
 		  - M3_STOP -> PD11
		  
		  - M4_DIR  -> PD8
 		  - M4_STOP -> PD9				
------------------------------------------*/								   
void PWM_GPIO_Config(void) 	  
{
	 GPIO_InitTypeDef GPIO_InitStructure;		  
	 
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,  ENABLE); 	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	  
	 GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/*------------------------------------------
 ��������:����TIM2�����PWM�źŵ�ģʽ
 ����˵��:- TIM2ͨ��4���PWM
 		  - PWMģʽ1
 		  - ���Ե͵�ƽ
		  - PWMƵ�� = 24kHz				
------------------------------------------*/
void PWM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
		  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 	

	TIM_TimeBaseStructure.TIM_Prescaler =71; 			        //ʱ��Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseStructure.TIM_Period = 999;				    //�Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	    //ʱ�ӷ�Ƶ1
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode =  TIM_OCMode_PWM2;         
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_Pulse = 0;     
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode =  TIM_OCMode_PWM2;         
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_Pulse = 0;     
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);				        
	
	TIM_OCInitStructure.TIM_OCMode =  TIM_OCMode_PWM2;           
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_Pulse = 0;     
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);	
	
	TIM_OCInitStructure.TIM_OCMode =  TIM_OCMode_PWM2;           
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_Pulse = 0;     
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);			         
	
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable); //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable); //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable); //ʹ��TIM4��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable); //ʹ��TIM4��CCR4�ϵ�Ԥװ�ؼĴ���

	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);

	TIM_Cmd(TIM4,ENABLE);	//ʹ��TIM4
}
/*------------------------------------------
 ��������:����˶�����ܽ�����
 ����˵��:- M1_DIR  -> PD14
				 - M1_STOP -> PD12

				 - M2_DIR  -> PD15
				 - M2_STOP -> PD13
				
				 - M3_DIR  -> PD10
				 - M3_STOP -> PD11
				
				 - M4_DIR  -> PD8
				 - M4_STOP -> PD9					
------------------------------------------*/ 
void MOTOR_DIR_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   // �������    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	M1_Forward;
	M2_Forward;
	M3_Forward;
	M4_Forward;
	
	M1_STOP;
	M2_STOP;
	M3_STOP;
	M4_STOP;	 
}
/*------------------------------------------
 ��������:PWM�����ʼ��				
------------------------------------------*/
void PWM_Init(void) 
{
	PWM_GPIO_Config();
	PWM_Mode_Config();
	MOTOR_DIR_GPIO_Config();		 
}
/*------------------------------------------
 ��������:���1�������˶�
 ��������:CCR2_VALռ�ձȼ���ֵ
 ����˵��:CCR2_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M1_Forward(uint16_t val)
{   	
	M1_Backward;
	M1_STOP;
	TIM_SetCompare2(TIM4,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���1�������˶�
 ��������:CCR2_VALռ�ձȼ���ֵ
 ����˵��:CCR2_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M1_Backward(uint16_t val)
{   	
	M1_Forward;
	M1_RELEASE;
	TIM_SetCompare2(TIM4,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���2�������˶�
 ��������:CCR3_VALռ�ձȼ���ֵ
 ����˵��:CCR3_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M2_Forward(uint16_t val)
{   
	M2_Backward;
	M2_STOP;
	TIM_SetCompare3(TIM4,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���2�������˶�
 ��������:CCR3_VALռ�ձȼ���ֵ
 ����˵��:CCR3_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M2_Backward(uint16_t val)
{   	
	M2_Forward;
	M2_RELEASE;
	TIM_SetCompare3(TIM4,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���3�������˶�
 ��������:CCR4_VALռ�ձȼ���ֵ
 ����˵��:CCR4_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M3_Forward(uint16_t val)
{   	
	M3_Forward;
	M3_RELEASE;
	TIM_SetCompare4(TIM4,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���3�������˶�
 ��������:CCR4_VALռ�ձȼ���ֵ
 ����˵��:CCR4_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M3_Backward(uint16_t val)
{   
	M3_Backward;
	M3_STOP;
	TIM_SetCompare4(TIM4,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���4�������˶�
 ��������:CCR1_VALռ�ձȼ���ֵ
 ����˵��:CCR1_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M4_Forward(uint16_t val)
{   
	M4_Forward;
	M4_RELEASE;
	TIM_SetCompare1(TIM4,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���4�������˶�
 ��������:CCR1_VALռ�ձȼ���ֵ
 ����˵��:CCR1_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M4_Backward(uint16_t val)
{   	
	M4_Backward;
	M4_STOP;
	TIM_SetCompare1(TIM4,val);     //ֵԽ��ת��Խ��
}
