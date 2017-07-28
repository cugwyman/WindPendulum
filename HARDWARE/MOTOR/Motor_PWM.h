#ifndef __MOTOR__PWM__H__
#define __MOTOR__PWM__H__	

#include "stm32f10x.h"


/* ����1��ʾ��ǰ,0��ʾ��� */
#define M1_Forward	 	GPIOD->BSRR = GPIO_Pin_14
#define M1_Backward	 	GPIOD->BRR  = GPIO_Pin_14

/* ����1��ʾ��ǰ,0��ʾ��� */
#define M2_Forward	 	GPIOD->BSRR = GPIO_Pin_15 
#define M2_Backward	 	GPIOD->BRR  = GPIO_Pin_15

/* ����1��ʾ��ǰ,0��ʾ��� */
#define M3_Forward	 	GPIOD->BSRR = GPIO_Pin_10
#define M3_Backward	 	GPIOD->BRR  = GPIO_Pin_10

/* ����1��ʾ��ǰ,0��ʾ��� */
#define M4_Forward	 	GPIOD->BSRR = GPIO_Pin_8
#define M4_Backward	 	GPIOD->BRR  = GPIO_Pin_8



/* ���1��ͨ��2 */
#define M1_STOP			GPIOD->BSRR = GPIO_Pin_12
#define M1_RELEASE      GPIOD->BRR  = GPIO_Pin_12

/* ���2��ͨ��3 */
#define M2_STOP			GPIOD->BSRR = GPIO_Pin_13
#define M2_RELEASE      GPIOD->BRR  = GPIO_Pin_13

/* ���3 */
#define M3_STOP			GPIOD->BSRR = GPIO_Pin_11
#define M3_RELEASE      GPIOD->BRR  = GPIO_Pin_11

/* ���4 */
#define M4_STOP			GPIOD->BSRR = GPIO_Pin_9
#define M4_RELEASE      GPIOD->BRR  = GPIO_Pin_9

void PWM_GPIO_Config(void);

void PWM_M1_Backward(uint16_t val);
void PWM_M1_Forward(uint16_t val);

void PWM_M2_Backward(uint16_t val);
void PWM_M2_Forward(uint16_t val);

void PWM_M3_Backward(uint16_t val);
void PWM_M3_Forward(uint16_t val);

void PWM_M4_Backward(uint16_t val);
void PWM_M4_Forward(uint16_t val);

void PWM_GPIO_Config(void);
void PWM_Init(void);
void PWM_Mode_Config(void);


#endif
