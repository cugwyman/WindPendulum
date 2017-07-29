#include "stm32f10x.h"
#include "uart6050.h"
#include "Motor_Control.h"
#include "Motor_PWM.h"
#include "Motor_PID.h"
#include "Timer.h"
#include "OscilloScope.h"

/**********************************************
 *
 *       Wind pendulum 风力摆 V2.0
 *               2017.7.29
 *     CUG     Wyman    Chen    Sun 
 *          
 *     IDE              Keil MDK V5.23
 *     MCU              STM32F103ZET6
 *     Attitude transducer     MPU6050
 *     Motor drivers    TB6612
 *     Motor            Coreless motor
 *     
 *     To be added      OLED, Keyboard
 *
 *     Thanks to motor part coder: BoX
 *
 **********************************************/
 
/**********************************************
 *
 *  2017.7.28   V1.0
 *  
 *  1.文件归属重新安排，外设均在HARDWARE文件夹
 *  2.全局变量统一在其定义处的C文件自身的h文件处进行外部引用，
 *    如M1TypeDef M1;在Motor_Control.c，则extern M1TypeDef M1;在Motor_Control.h
 *  3.初始化角度测零点时，直接用中断读取量，而非for循环读取，未知优劣
 *  4.修改部分命名、格式问题，修正所有warning
 *  5.Motor_Control.c处所有模式的set_x及set_y使用统一的全局变量，方便示波器调用
 *  6.Motor_Control.h处的限幅只使用了POWER_MAX，大小可修改
 *
 *  2017.7.29   V2.0
 *  
 *  1.修改电机工作方式，每个方向有单独PWM控制，已经修正死区电压
 *  2.重新测量摆动周期，设置摆长、半径等
 *  3.修改PID参数，控制较好，所有功能已在允许误差范围内
 *
 **********************************************/
 
int main()
{
	delay_init();
	uart_init(115200);
	UART6050_Init();
	PID_M1_Init();
	PID_M2_Init();
    M1TypeDef_Init();
	M2TypeDef_Init();
    PWM_Init();
    
    CurMode = 5;//风力摆控制模式选择
    
	TIM5_Config(5000-1,72-1);// TIM5 5ms Inturrupt 采样率200Hz
	while(1)
	{
		OutPut_Data(M2.CurPos, set_y, M1.CurPos, set_x);// Y轴检测，Y轴设定，X轴检测，X轴设定
//        printf("%4.2f, %4.2f\n", M1.CurPos, M2.CurPos);
		delay_ms(50);
	}
}

