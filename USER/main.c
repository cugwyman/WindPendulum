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

