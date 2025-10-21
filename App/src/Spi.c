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
 * @file spi.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "n32g430.h"
#include "stdio.h"
#include "string.h"
#include "bsp_delay.h"
#include "SystemDefine.h"
#include "MotorDrive.h"

void Spi_init(void);					
void DAC_update(uint8_t num,uint16_t data);
// ========================================================================
// 函数名称：Spi_init
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：SPI初始化
// ========================================================================
void Spi_init(void)
{
	  SPI_I2S_Reset(SPI2);
		SPI_InitType SPI_InitStructure;
	
		SPI_Initializes_Structure(&SPI_InitStructure);
		SPI_InitStructure.DataDirection = SPI_DIR_SINGLELINE_TX;		//发送模式 单工模式
		SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;					//主模式
		SPI_InitStructure.DataLen       = SPI_DATA_SIZE_16BITS;			//数据16位宽
		SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW; 					//时钟默认电平是低
		SPI_InitStructure.CLKPHA        = SPI_CLKPHA_SECOND_EDGE;		//第二个边沿采集数据
		SPI_InitStructure.NSS           = SPI_NSS_HARD;							//硬件NSS
		SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_4;			  //时钟分频
		SPI_InitStructure.FirstBit      = SPI_FB_MSB;								//MSB
		SPI_InitStructure.CRCPoly       = 7;
		SPI_Initializes(SPI2, &SPI_InitStructure);
		SPI_SS_Output_Enable(SPI2);																	//NSS 使能

		/* 发送字符唤醒MAX5742芯片 */
		SPI_ON(SPI2);
		SPI_I2S_Data_Transmit(SPI2, 0xfffc);	
		while(SPI_I2S_Flag_Status_Get(SPI2, SPI_I2S_FLAG_TE) == RESET)
				;   
		SPI_OFF(SPI2);
}
// ========================================================================
// 函数名称：DAC_update
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：DAC数据发送
// ========================================================================
void DAC_update(uint8_t num,uint16_t data)
{
	TIM3_Delay_Us(1);	
	SPI_ON(SPI2);
	switch(num)
	{
		case 1 :
			 data=data|0x0000;		//DAC_1 通道
			 SPI_I2S_Data_Transmit(SPI2, data);	
			 while(SPI_I2S_Flag_Status_Get(SPI2, SPI_I2S_FLAG_TE) == RESET){}
			 break;
		case 2 :
			 data=data|0x1000;		//DAC_2 通道
			 SPI_I2S_Data_Transmit(SPI2, data);	
			 while(SPI_I2S_Flag_Status_Get(SPI2, SPI_I2S_FLAG_TE) == RESET){}
			 break;
		case 3 :
			 data=data|0x2000;		//DAC_3 通道
			 SPI_I2S_Data_Transmit(SPI2, data);	
			 while(SPI_I2S_Flag_Status_Get(SPI2, SPI_I2S_FLAG_TE) == RESET){}
			 break;
		case 4 :
			 data=data|0x3000;		//DAC_4 通道
			 SPI_I2S_Data_Transmit(SPI2, data);	
			 while(SPI_I2S_Flag_Status_Get(SPI2, SPI_I2S_FLAG_TE) == RESET){}
			 break;
		default:
			 break;
	}
	SPI_OFF(SPI2);		
}
