#include "myiic.h"
#include "bsp_delay.h"
	

//初始化IIC
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
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA_SET();	  	  
	IIC_SCL_SET();
	SysTick_Delay_Us(4);
 	IIC_SDA_RESET();//START:when CLK is high,DATA change form high to low 
	SysTick_Delay_Us(4);
	IIC_SCL_RESET();//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL_RESET();
	IIC_SDA_RESET();//STOP:when CLK is high DATA change form low to high
 	SysTick_Delay_Us(4);
	IIC_SCL_SET(); 
	IIC_SDA_SET();//发送I2C总线结束信号
	SysTick_Delay_Us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
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
	IIC_SCL_RESET();//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL_RESET();//拉低时钟开始数据传输
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
		SysTick_Delay_Us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL_SET();
		SysTick_Delay_Us(2); 
		IIC_SCL_RESET();	
		SysTick_Delay_Us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}



























