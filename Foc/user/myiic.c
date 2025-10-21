#include "myiic.h"
#include "bsp_delay.h"
	

//��ʼ��IIC
void IIC_Init(void)
{				
	GPIO_InitType GPIO_InitStructure;
	RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);

	GPIO_Structure_Initialize(&GPIO_InitStructure);
	/*PB13 -- SCL; PB14 -- SDA*/
	GPIO_InitStructure.Pin            = GPIO_PIN_13|GPIO_PIN_14 ;
	GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST; 
	GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUT_PP;
	GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP; 
	GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
	
	IIC_SCL_SET();
	IIC_SDA_SET();
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA_SET();	  	  
	IIC_SCL_SET();
	SysTick_Delay_Us(4);
 	IIC_SDA_RESET();//START:when CLK is high,DATA change form high to low 
	SysTick_Delay_Us(4);
	IIC_SCL_RESET();//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL_RESET();
	IIC_SDA_RESET();//STOP:when CLK is high DATA change form low to high
 	SysTick_Delay_Us(4);
	IIC_SCL_SET(); 
	IIC_SDA_SET();//����I2C���߽����ź�
	SysTick_Delay_Us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA_SET();SysTick_Delay_Us(1);	   
	IIC_SCL_SET();SysTick_Delay_Us(1);	 
	while(READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_RESET();//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL_RESET();
	SDA_OUT();
	IIC_SDA_RESET();
	SysTick_Delay_Us(2);
	IIC_SCL_SET();
	SysTick_Delay_Us(2);
	IIC_SCL_RESET();
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL_RESET();
	SDA_OUT();
	IIC_SDA_SET();
	SysTick_Delay_Us(2);
	IIC_SCL_SET();
	SysTick_Delay_Us(2);
	IIC_SCL_RESET();
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL_RESET();//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {   
				if((txd&0x80)>>7)
				{
					IIC_SDA_SET();
				}
				else
				{
					IIC_SDA_RESET();
				}
        txd<<=1; 	  
		SysTick_Delay_Us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_SET();
		SysTick_Delay_Us(2); 
		IIC_SCL_RESET();	
		SysTick_Delay_Us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL_RESET(); 
        SysTick_Delay_Us(2);
		IIC_SCL_SET();
        receive<<=1;
        if(READ_SDA())receive++;   
		SysTick_Delay_Us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



























