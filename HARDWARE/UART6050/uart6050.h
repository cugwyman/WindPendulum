#ifndef __UART6050_H
#define __UART6050_H

#include "string.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"

void Uart2_NVIC(void) ;   //初始化 配置USART2
void Read_MPUData(void);    //读取MPU的值
void NVIC_Configuration(void);//uart2中断初始化
void UART6050_Init(void);//串口陀螺仪初始

extern float pitch_zero, roll_zero;
extern float a[3],w[3],angle[3],T;

#endif	   

