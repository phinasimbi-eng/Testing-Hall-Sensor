/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file uart.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "SystemDefine.h"
#include "stdio.h"
#include "MotorDrive.h"
#include "string.h"

extern RecvFrame mRecvFrame;
void Uart_init(void);
void Send_Value_Uart(uint8_t * pChar,int32_t len);
uint8_t SxBuffer1[BUFFSIZE],RxBuffer1[BUFFSIZE1];

// ========================================================================
// 函数名称：Uart_init
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：调试串口初始化
// ========================================================================
void Uart_DMA_config(void)
{

    DMA_InitType DMA_InitStructure;

    /* 使用DMA接收数据配置 */
    DMA_Reset(DMA_CH4);
    DMA_InitStructure.PeriphAddr     = (uint32_t)&(UART3->DAT);				//串口数据寄存器
    DMA_InitStructure.MemAddr        = (uint32_t)RxBuffer1;						//接收数据缓冲区
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;						//数据传输方向
    DMA_InitStructure.BufSize        = BUFFSIZE1;											//缓冲区大小
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;		//外设地址不增加
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;				//内存地址自动增加
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;		//外设数据宽度
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_BYTE;				//内存数据宽度
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_DISABLE;			//循环传输失能
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGH;					//dma优先级
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;						//不是内存到内存的传输方式
    DMA_Initializes(DMA_CH4, &DMA_InitStructure);											//初始化DMA
	
    DMA_Channel_Request_Remap(DMA_CH4, DMA_REMAP_UART3_RX);						//DMA 通道映射
		DMA_Interrupts_Enable(DMA_CH4, DMA_INT_TXC);											//DMA 传输完成中断使能
    USART_DMA_Transfer_Enable(UART3, USART_DMAREQ_RX );								//串口DMA 接收使能
    DMA_Channel_Enable(DMA_CH4);																			//DMA使能
		
		/* 使用DMA发送数据配置 */
		DMA_Reset(DMA_CH3);
    DMA_InitStructure.PeriphAddr     = (uint32_t)&(UART3->DAT);				//串口数据寄存器
    DMA_InitStructure.MemAddr        = (uint32_t)SxBuffer1;						//发送数据缓冲区
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;						//数据传输方向
    DMA_InitStructure.BufSize        = BUFFSIZE;											//缓冲区大小
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;		//外设地址不增加
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;				//内存地址自动增加
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;		//外设数据宽度
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_BYTE;				//内存数据宽度
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_DISABLE;			//循环传输失能
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGHEST;				//dma优先级
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;						//不是内存到内存的传输方式
    DMA_Initializes(DMA_CH3, &DMA_InitStructure);											//初始化DMA
	
    DMA_Channel_Request_Remap(DMA_CH3, DMA_REMAP_UART3_TX);						//DMA 通道映射
		DMA_Interrupts_Enable(DMA_CH3, DMA_INT_TXC);											//DMA 传输完成中断使能
    USART_DMA_Transfer_Enable(UART3, USART_DMAREQ_TX );								//串口DMA发送使能
    //DMA_Channel_Enable(DMA_CH3);																			//DMA使能		
}
// ========================================================================
// 函数名称：Uart_init
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：调试串口初始化
// ========================================================================
void Uart_init(void)
{
    USART_InitType USART_InitStructure;
    NVIC_InitType NVIC_InitStructure;
    
    USART_InitStructure.BaudRate = 4000000l;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.Parity = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
    USART_Initializes(UART3,&USART_InitStructure);

//    NVIC_InitStructure.NVIC_IRQChannel = UART3_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Initializes(&NVIC_InitStructure);
//	
//		USART_Interrput_Enable(UART3,USART_INT_RXDNE);
    USART_Enable(UART3);

		/*dma 中断发送使能*/
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA_Channel4_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

		/*dma 数据发送完成中断使能*/
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA_Channel3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

		Uart_DMA_config();
}
// ========================================================================
// 函数名称：Send_Value_Uart
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：串口发送数据处理
// ========================================================================
void Send_Value_Uart(uint8_t * pChar,int32_t len)
{
	uint16_t i = 0;
	for(i = 0;i<len;i++) 
    {
        USART_Data_Send(UART3,pChar[i]);
        while(USART_Flag_Status_Get(UART3, USART_FLAG_TXDE) == RESET)
            ;
	}
}
// ========================================================================
// 函数名称：printf
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：串口打印数据
// ========================================================================
int fputc(int ch, FILE *f)
{      
	USART_Data_Send(UART3, (uint8_t) ch);
  while(USART_Flag_Status_Get(UART3, USART_FLAG_TXDE) == RESET)
            ;
	return (ch);
}
