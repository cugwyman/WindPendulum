#include "uart6050.h"

float a[3],w[3],angle[3],T;
unsigned char Re_buf[11],temp_buf[11],counter = 0;
unsigned char sign,Temp[11];
float pitch_zero = 0, roll_zero = 0;

void USART2_Config(void)   //初始化 配置USART2
{
	GPIO_InitTypeDef    GPIO_InitStructure;	   //串口端口配置结构体变量
	USART_InitTypeDef   USART_InitStructure;   //串口参数配置结构体变量

	//使能 USART2 时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//打开串口复用时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);   //打开PA端口时钟

	//将USART2 Tx（发送脚）的GPIO配置为推挽复用模式   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				  //选定哪个IO口 现选定PA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;           //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		  //设定IO口的输出速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);    				  //初始化GPIOA

	//将USART2 Rx（接收脚）的GPIO配置为浮空输入模式														  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				  //选定哪个IO口 现选定PA3
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  //浮空输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);                    //初始化GPIOA
	  
	//配置USART2参数
	USART_InitStructure.USART_BaudRate = 115200;	                    //波特率设置：115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	    //数据位数设置：8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	        //停止位设置：1位
	USART_InitStructure.USART_Parity = USART_Parity_No ;            //是否奇偶校验：无
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制模式设置：没有使能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //接收与发送都使能
	USART_Init(USART2, &USART_InitStructure);                       //初始化USART2
	
	//打开发送中断和接收中断(如果需要中断)
	//USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  // 使能指定的USART2发送中断
  	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // 使能指定的USART2接收中断

	//使能 USART2， 配置完毕
	USART_Cmd(USART2, ENABLE);                             // USART2使能

	//如下语句解决第1个字节无法正确发送出去的问题
    USART_ClearFlag(USART2, USART_FLAG_TC);                //清串口2发送标志
}

void USART2_IRQHandler(void)		   //串口2全局中断服务函数
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断有效,若接收数据寄存器满
    {
        Temp[counter] = USART_ReceiveData(USART2);   //接收数据
        if(counter == 0 && Temp[0] != 0x55) return;      //第 0 号数据不是帧头，跳过
        counter++; 
        if(counter==11) //接收到 11 个数据
        { 
        memcpy(Re_buf,Temp,11);// 由Temp所指内存区域复制11个字节到Re_buf所指的内存区域
        counter=0; //重新赋值，准备下一帧数据的接收
        sign=1;
        }    
    }
    Read_MPUData(); 
}


void Uart2_NVIC(void)   //uart2中断
{
    NVIC_InitTypeDef    NVIC_InitStructure; 					   //定义用于配置中断的结构体变量

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                //中断优先级分组  抢占式优先级别设置为2位；响应优先级占2位

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			   //指定中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	   //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;             //指定响应优先级别1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	               //打开中断
	NVIC_Init(&NVIC_InitStructure);				
}


void Read_MPUData(void)
{
	if(sign)
    {  
        memcpy(Temp,Re_buf,11);
        sign=0;
        if(Re_buf[0]==0x55)       //检查帧头
        {  
            switch(Re_buf[1])
            {
                case 0x51: //标识这个包是加速度包
                    a[0] = ((short)(Temp[3]<<8 | Temp[2]))/32768.0*16;       //X轴加速度
                    a[1] = ((short)(Temp[5]<<8 | Temp[4]))/32768.0*16;       //Y轴加速度
                    a[2] = ((short)(Temp[7]<<8 | Temp[6]))/32768.0*16;       //Z轴加速度
                    T    = ((short)(Temp[9]<<8 | Temp[8]))/340.0+36.25;      //温度
                    break;
                case 0x52: //标识这个包是角速度包
                    w[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*2000;      //X轴角速度
                    w[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*2000;      //Y轴角速度
                    w[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*2000;      //Z轴角速度
                    T    = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;       //温度
                    break;
                case 0x53: //标识这个包是角度包
                    angle[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*180;   //X轴滚转角（x 轴）
                    angle[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*180;   //Y轴俯仰角（y 轴）
                    angle[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*180;   //Z轴偏航角（z 轴）
                    T        = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;   //温度
                    break;
                default:  break;
            }
//        printf("AgX: %2.2f  AgY: %2.2f  AgZ :%2.2f   Vx:%2.2f  Vy:%.2f  Az:%.2f   Ax:%2.2f  Ay:%2.2f  Az:%2.2f\r\n",angle[0],angle[1],angle[2],w[0],w[1],w[2],a[0],a[1],a[2]);
//        printf("AgX: %2.2f AgY: %2.2f AgZ :%2.2f T:%2.2f \r\n",angle[0],angle[1],angle[2],T);//发送角度
//        printf("WX: %2.2f WY: %2.2f WZ :%2.2f T:%2.2f \r\n",w[0],w[1],w[2],T);//发送角速度
//        printf("aX: %2.2f aY: %2.2f aZ :%2.2f T:%2.2f \r\n",a[0],a[1],a[2],T);//发送角加速度				
        }
    }
}


void UART6050_Init(void)
{
    int i;
	USART2_Config();//串口2初始化
	Uart2_NVIC() ;//串口中断
    for(i = 0; i < 1000; ++i)
    {
//        Read_MPUData();
        pitch_zero += angle[0];
        roll_zero += angle[1];
        if(i > 0)
        {
            pitch_zero /= 2;
            roll_zero /= 2;
        }
		delay_ms(1);
    }
}













