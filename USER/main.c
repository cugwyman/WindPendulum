#include "stm32f10x.h"
#include "uart6050.h"
#include "Motor_Control.h"
#include "Motor_PWM.h"
#include "Motor_PID.h"
#include "Timer.h"
#include "OscilloScope.h"

/**********************************************
 *
 *       Wind pendulum ������ V2.0
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
 *  1.�ļ��������°��ţ��������HARDWARE�ļ���
 *  2.ȫ�ֱ���ͳһ���䶨�崦��C�ļ������h�ļ��������ⲿ���ã�
 *    ��M1TypeDef M1;��Motor_Control.c����extern M1TypeDef M1;��Motor_Control.h
 *  3.��ʼ���ǶȲ����ʱ��ֱ�����ж϶�ȡ��������forѭ����ȡ��δ֪����
 *  4.�޸Ĳ�����������ʽ���⣬��������warning
 *  5.Motor_Control.c������ģʽ��set_x��set_yʹ��ͳһ��ȫ�ֱ���������ʾ��������
 *  6.Motor_Control.h�����޷�ֻʹ����POWER_MAX����С���޸�
 *
 *  2017.7.29   V2.0
 *  
 *  1.�޸ĵ��������ʽ��ÿ�������е���PWM���ƣ��Ѿ�����������ѹ
 *  2.���²����ڶ����ڣ����ðڳ����뾶��
 *  3.�޸�PID���������ƽϺã����й�������������Χ��
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
    
    CurMode = 5;//�����ڿ���ģʽѡ��
    
	TIM5_Config(5000-1,72-1);// TIM5 5ms Inturrupt ������200Hz
	while(1)
	{
		OutPut_Data(M2.CurPos, set_y, M1.CurPos, set_x);// Y���⣬Y���趨��X���⣬X���趨
//        printf("%4.2f, %4.2f\n", M1.CurPos, M2.CurPos);
		delay_ms(50);
	}
}

