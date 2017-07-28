#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "stm32f10x.h"
/*------------------------------------------
 			  电机物理限制				
------------------------------------------*/
#define STROKE_LIMIT 	 (90000)   //物理极限行程<0-80000>
#define SPEED_LIMIT  	 (65535)   //物理极限速度<65535>
#define POWER_LIMIT  	 (5000)    //物理极限功率<0-3000>

#define STROKE_MAX   	 (80000)   //软件极限行程<0-80000>
#define SPEED_MAX	 	 (300)	   //软件极限速度<0-300>
#define POWER_MAX	 	 (5000)	   //软件极限功率<0-3000>

#define TIMEOUT_CNT  	 (5000)	   //归零超时计数器ms<0-10000>
#define ZERO_POINT	 	 (10000)   //设置参考零点为<0-80000>
/*------------------------------------------
 			  电机启动默认值				
------------------------------------------*/
#define DEFAULT_SPEED	 (10)	   
#define DEFAULT_POSITION (11000)
#define DEFAULT_POWER    (200)
/*------------------------------------------
 				电机结构体				
------------------------------------------*/
typedef struct
{
	float Offset;	  //允许偏差量
	float CurPos;
	float PrevPos;
	float CurAcc;
	float PrevSpeed;

	volatile float SetXPos;	  //设定位置
	volatile float SetYPos;	  //设定位置
	volatile float SetSpeed;  //设定速度
	
	volatile float CurXPos;	  //当前位置
	volatile float CurYPos;	  //当前位置
	volatile float CurSpeed;  //当前速度矢量

	volatile int32_t  PWM;	      //PWM
	volatile uint8_t  ShootFlag;
	volatile uint8_t  AdjustFlag;
	volatile uint8_t  ErrFlag;

	volatile uint32_t SetMaxPos;	  //软件设定最大位置
	volatile uint32_t SetMaxPower;	  //软件设定最大力量
	volatile int32_t  SetMaxSpeed;	  //软件设定最大速度
		
}M1TypeDef,M2TypeDef;

extern M1TypeDef M1;
extern M2TypeDef M2;
extern float set_y;
extern float set_x;

void MCU_Reset(void);
void M1TypeDef_Init(void);
void M2TypeDef_Init(void);
void Mode_0(void);
void Mode_1(void);
void Mode_2(void);
void Mode_3(void);
void Mode_4(void);
void Mode_5(void);
void Mode_6(void);
void MotorMove(int32_t pwm1,int32_t pwm2);
#endif

