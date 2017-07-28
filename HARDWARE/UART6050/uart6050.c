#include "uart6050.h"

float a[3],w[3],angle[3],T;
unsigned char Re_buf[11],temp_buf[11],counter = 0;
unsigned char sign,Temp[11];
float pitch_zero = 0, roll_zero = 0;

void USART2_Config(void)   //��ʼ�� ����USART2
{
	GPIO_InitTypeDef    GPIO_InitStructure;	   //���ڶ˿����ýṹ�����
	USART_InitTypeDef   USART_InitStructure;   //���ڲ������ýṹ�����

	//ʹ�� USART2 ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//�򿪴��ڸ���ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);   //��PA�˿�ʱ��

	//��USART2 Tx�����ͽţ���GPIO����Ϊ���츴��ģʽ   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				  //ѡ���ĸ�IO�� ��ѡ��PA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;           //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		  //�趨IO�ڵ�����ٶ�Ϊ50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);    				  //��ʼ��GPIOA

	//��USART2 Rx�����սţ���GPIO����Ϊ��������ģʽ														  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				  //ѡ���ĸ�IO�� ��ѡ��PA3
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  //��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);                    //��ʼ��GPIOA
	  
	//����USART2����
	USART_InitStructure.USART_BaudRate = 115200;	                    //���������ã�115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	    //����λ�����ã�8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	        //ֹͣλ���ã�1λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;            //�Ƿ���żУ�飺��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//Ӳ��������ģʽ���ã�û��ʹ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //�����뷢�Ͷ�ʹ��
	USART_Init(USART2, &USART_InitStructure);                       //��ʼ��USART2
	
	//�򿪷����жϺͽ����ж�(�����Ҫ�ж�)
	//USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  // ʹ��ָ����USART2�����ж�
  	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // ʹ��ָ����USART2�����ж�

	//ʹ�� USART2�� �������
	USART_Cmd(USART2, ENABLE);                             // USART2ʹ��

	//�����������1���ֽ��޷���ȷ���ͳ�ȥ������
    USART_ClearFlag(USART2, USART_FLAG_TC);                //�崮��2���ͱ�־
}

void USART2_IRQHandler(void)		   //����2ȫ���жϷ�����
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж���Ч,���������ݼĴ�����
    {
        Temp[counter] = USART_ReceiveData(USART2);   //��������
        if(counter == 0 && Temp[0] != 0x55) return;      //�� 0 �����ݲ���֡ͷ������
        counter++; 
        if(counter==11) //���յ� 11 ������
        { 
        memcpy(Re_buf,Temp,11);// ��Temp��ָ�ڴ�������11���ֽڵ�Re_buf��ָ���ڴ�����
        counter=0; //���¸�ֵ��׼����һ֡���ݵĽ���
        sign=1;
        }    
    }
    Read_MPUData(); 
}


void Uart2_NVIC(void)   //uart2�ж�
{
    NVIC_InitTypeDef    NVIC_InitStructure; 					   //�������������жϵĽṹ�����

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                //�ж����ȼ�����  ��ռʽ���ȼ�������Ϊ2λ����Ӧ���ȼ�ռ2λ

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			   //ָ���ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	   //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;             //ָ����Ӧ���ȼ���1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	               //���ж�
	NVIC_Init(&NVIC_InitStructure);				
}


void Read_MPUData(void)
{
	if(sign)
    {  
        memcpy(Temp,Re_buf,11);
        sign=0;
        if(Re_buf[0]==0x55)       //���֡ͷ
        {  
            switch(Re_buf[1])
            {
                case 0x51: //��ʶ������Ǽ��ٶȰ�
                    a[0] = ((short)(Temp[3]<<8 | Temp[2]))/32768.0*16;       //X����ٶ�
                    a[1] = ((short)(Temp[5]<<8 | Temp[4]))/32768.0*16;       //Y����ٶ�
                    a[2] = ((short)(Temp[7]<<8 | Temp[6]))/32768.0*16;       //Z����ٶ�
                    T    = ((short)(Temp[9]<<8 | Temp[8]))/340.0+36.25;      //�¶�
                    break;
                case 0x52: //��ʶ������ǽ��ٶȰ�
                    w[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*2000;      //X����ٶ�
                    w[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*2000;      //Y����ٶ�
                    w[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*2000;      //Z����ٶ�
                    T    = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;       //�¶�
                    break;
                case 0x53: //��ʶ������ǽǶȰ�
                    angle[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*180;   //X���ת�ǣ�x �ᣩ
                    angle[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*180;   //Y�ḩ���ǣ�y �ᣩ
                    angle[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*180;   //Z��ƫ���ǣ�z �ᣩ
                    T        = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;   //�¶�
                    break;
                default:  break;
            }
//        printf("AgX: %2.2f  AgY: %2.2f  AgZ :%2.2f   Vx:%2.2f  Vy:%.2f  Az:%.2f   Ax:%2.2f  Ay:%2.2f  Az:%2.2f\r\n",angle[0],angle[1],angle[2],w[0],w[1],w[2],a[0],a[1],a[2]);
//        printf("AgX: %2.2f AgY: %2.2f AgZ :%2.2f T:%2.2f \r\n",angle[0],angle[1],angle[2],T);//���ͽǶ�
//        printf("WX: %2.2f WY: %2.2f WZ :%2.2f T:%2.2f \r\n",w[0],w[1],w[2],T);//���ͽ��ٶ�
//        printf("aX: %2.2f aY: %2.2f aZ :%2.2f T:%2.2f \r\n",a[0],a[1],a[2],T);//���ͽǼ��ٶ�				
        }
    }
}


void UART6050_Init(void)
{
    int i;
	USART2_Config();//����2��ʼ��
	Uart2_NVIC() ;//�����ж�
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













