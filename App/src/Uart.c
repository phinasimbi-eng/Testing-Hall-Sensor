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
// �������ƣ�Uart_init
// �����������
// �����������
// ��    �룺��
// ��    ������
// �������������Դ��ڳ�ʼ��
// ========================================================================
void Uart_DMA_config(void)
{

    DMA_InitType DMA_InitStructure;

    /* ʹ��DMA������������ */
    DMA_Reset(DMA_CH4);
    DMA_InitStructure.PeriphAddr     = (uint32_t)&(UART3->DAT);				//�������ݼĴ���
    DMA_InitStructure.MemAddr        = (uint32_t)RxBuffer1;						//�������ݻ�����
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;						//���ݴ��䷽��
    DMA_InitStructure.BufSize        = BUFFSIZE1;											//��������С
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;		//�����ַ������
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;				//�ڴ��ַ�Զ�����
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;		//�������ݿ��
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_BYTE;				//�ڴ����ݿ��
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_DISABLE;			//ѭ������ʧ��
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGH;					//dma���ȼ�
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;						//�����ڴ浽�ڴ�Ĵ��䷽ʽ
    DMA_Initializes(DMA_CH4, &DMA_InitStructure);											//��ʼ��DMA
	
    DMA_Channel_Request_Remap(DMA_CH4, DMA_REMAP_UART3_RX);						//DMA ͨ��ӳ��
		DMA_Interrupts_Enable(DMA_CH4, DMA_INT_TXC);											//DMA ��������ж�ʹ��
    USART_DMA_Transfer_Enable(UART3, USART_DMAREQ_RX );								//����DMA ����ʹ��
    DMA_Channel_Enable(DMA_CH4);																			//DMAʹ��
		
		/* ʹ��DMA������������ */
		DMA_Reset(DMA_CH3);
    DMA_InitStructure.PeriphAddr     = (uint32_t)&(UART3->DAT);				//�������ݼĴ���
    DMA_InitStructure.MemAddr        = (uint32_t)SxBuffer1;						//�������ݻ�����
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;						//���ݴ��䷽��
    DMA_InitStructure.BufSize        = BUFFSIZE;											//��������С
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;		//�����ַ������
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;				//�ڴ��ַ�Զ�����
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;		//�������ݿ��
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_BYTE;				//�ڴ����ݿ��
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_DISABLE;			//ѭ������ʧ��
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGHEST;				//dma���ȼ�
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;						//�����ڴ浽�ڴ�Ĵ��䷽ʽ
    DMA_Initializes(DMA_CH3, &DMA_InitStructure);											//��ʼ��DMA
	
    DMA_Channel_Request_Remap(DMA_CH3, DMA_REMAP_UART3_TX);						//DMA ͨ��ӳ��
		DMA_Interrupts_Enable(DMA_CH3, DMA_INT_TXC);											//DMA ��������ж�ʹ��
    USART_DMA_Transfer_Enable(UART3, USART_DMAREQ_TX );								//����DMA����ʹ��
    //DMA_Channel_Enable(DMA_CH3);																			//DMAʹ��		
}
// ========================================================================
// �������ƣ�Uart_init
// �����������
// �����������
// ��    �룺��
// ��    ������
// �������������Դ��ڳ�ʼ��
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

		/*dma �жϷ���ʹ��*/
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA_Channel4_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

		/*dma ���ݷ�������ж�ʹ��*/
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA_Channel3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

		Uart_DMA_config();
}
// ========================================================================
// �������ƣ�Send_Value_Uart
// �����������
// �����������
// ��    �룺��
// ��    ������
// �������������ڷ������ݴ���
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
// �������ƣ�printf
// �����������
// �����������
// ��    �룺��
// ��    ������
// �������������ڴ�ӡ����
// ========================================================================
int fputc(int ch, FILE *f)
{      
	USART_Data_Send(UART3, (uint8_t) ch);
  while(USART_Flag_Status_Get(UART3, USART_FLAG_TXDE) == RESET)
            ;
	return (ch);
}
