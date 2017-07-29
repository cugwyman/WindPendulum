#include "Motor_Control.h"
#include "Motor_PWM.h"
#include "Motor_PID.h"
#include "Timer.h"
#include "stm32f10x.h"
#include "stdio.h"
#include "stdlib.h"
#include "uart6050.h"
/*------------------------------------------
 				全局变量				
------------------------------------------*/ 
uint8_t CurMode = 0;
/*-----------------------------------------------
 函数功能: TIM5定时器为PID采样计算提供稳定中断
 函数参数: ARR寄存器值0-65535,预分频值0-65535
 参 考 值: TIM5_Config(999,71)
	       计数频率1MHz,中断频率1000Hz
		   计数器每1us加1,中断每1ms产生一次		  			  
-----------------------------------------------*/
void TIM5_Config(unsigned short int Period,unsigned short int Prescaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	
	
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler; 			//时钟预分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseStructure.TIM_Period = Period;			        //自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	    //时钟分频1
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;			
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM5,TIM_FLAG_Update);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);  					

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //抢占优先级2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;   //响应优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	            
	NVIC_Init(&NVIC_InitStructure);		
	
	TIM_Cmd(TIM5,ENABLE); //使能TIMx		
}

/*-----------------------------------------------
 函数功能:TIM5中断服务程序
 函数说明:每5ms进入一次中断,采样率200Hz
 实测运行时间: 3.93ms
-----------------------------------------------*/
#define H (0.82f)  //万向节距地面的高度(米)
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
		//计算速度
		M1.CurSpeed = M1.CurPos - M1.PrevPos+0.04;
		M1.PrevPos = M1.CurPos;				
		
		M2.CurSpeed = M2.CurPos - M2.PrevPos;
		M2.PrevPos = M2.CurPos;	

        switch(CurMode)	//根据题目选择函数
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
