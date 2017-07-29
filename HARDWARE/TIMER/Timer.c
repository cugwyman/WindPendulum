#include "Motor_Control.h"
#include "Motor_PWM.h"
#include "Motor_PID.h"
#include "Timer.h"
#include "stm32f10x.h"
#include "stdio.h"
#include "stdlib.h"
#include "uart6050.h"
/*------------------------------------------
 				ȫ�ֱ���				
------------------------------------------*/ 
uint8_t CurMode = 0;
/*-----------------------------------------------
 ��������: TIM5��ʱ��ΪPID���������ṩ�ȶ��ж�
 ��������: ARR�Ĵ���ֵ0-65535,Ԥ��Ƶֵ0-65535
 �� �� ֵ: TIM5_Config(999,71)
	       ����Ƶ��1MHz,�ж�Ƶ��1000Hz
		   ������ÿ1us��1,�ж�ÿ1ms����һ��		  			  
-----------------------------------------------*/
void TIM5_Config(unsigned short int Period,unsigned short int Prescaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	
	
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler; 			//ʱ��Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseStructure.TIM_Period = Period;			        //�Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	    //ʱ�ӷ�Ƶ1
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;			
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM5,TIM_FLAG_Update);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);  					

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //��ռ���ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;   //��Ӧ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	            
	NVIC_Init(&NVIC_InitStructure);		
	
	TIM_Cmd(TIM5,ENABLE); //ʹ��TIMx		
}

/*-----------------------------------------------
 ��������:TIM5�жϷ������
 ����˵��:ÿ5ms����һ���ж�,������200Hz
 ʵ������ʱ��: 3.93ms
-----------------------------------------------*/
#define H (0.82f)  //����ھ����ĸ߶�(��)
void TIM5_IRQHandler(void)
{  	
	float pitch_temp1 = 0.0;
	float roll_temp1 = 0.0;
	GPIOE->BSRR = GPIO_Pin_3;
	if(TIM_GetITStatus(TIM5,TIM_IT_Update) == SET)
	{			
		pitch_temp1 =angle[0];
		roll_temp1  =angle[1];
		
        M1.CurPos = pitch_temp1 - pitch_zero;       
        M2.CurPos = roll_temp1 - roll_zero; 	
		//�����ٶ�
		M1.CurSpeed = M1.CurPos - M1.PrevPos+0.04;
		M1.PrevPos = M1.CurPos;				
		
		M2.CurSpeed = M2.CurPos - M2.PrevPos;
		M2.PrevPos = M2.CurPos;	

        switch(CurMode)	//������Ŀѡ����
		{	
			case 1: Mode_1(); break;
			case 2: Mode_2(); break;
			case 3: Mode_3(); break;
			case 4: Mode_4(); break;
			case 5: Mode_5(); break;
			case 6: Mode_6(); break;
			default:break;
		}
				
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);		
	}
	GPIOE->BRR = GPIO_Pin_3;					 	
}
