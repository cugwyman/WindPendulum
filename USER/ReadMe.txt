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