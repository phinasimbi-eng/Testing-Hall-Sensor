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
 * @file adc.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "SystemDefine.h"
#include "MotorDrive.h"
void Adc_Init(void);				// ADC初始化

extern __IO uint16_t ADC_RegularConvertedValueTab[8];
extern __IO uint16_t ADC_InjectedConvertedValueTab[4];
//DMA for adc requence 
void DMA_AdcSequenceIntiall(void)
{
    DMA_InitType DMA_InitStructure;
    DMA_Reset(DMA_CH1);
    DMA_InitStructure.PeriphAddr        = (uint32_t)&ADC->DAT;
    DMA_InitStructure.MemAddr           = (uint32_t)ADC_RegularConvertedValueTab;
    DMA_InitStructure.Direction         = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize           = 3;
    DMA_InitStructure.PeriphInc         = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.MemoryInc         = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.PeriphDataSize    = DMA_PERIPH_DATA_WIDTH_HALFWORD;
    DMA_InitStructure.MemDataSize       = DMA_MEM_DATA_WIDTH_HALFWORD;
    DMA_InitStructure.CircularMode      = DMA_CIRCULAR_MODE_ENABLE;
    DMA_InitStructure.Priority          = DMA_CH_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem           = DMA_MEM2MEM_DISABLE;
    DMA_Initializes(DMA_CH1, &DMA_InitStructure);
	DMA_Channel_Request_Remap(DMA_CH1, DMA_REMAP_ADC);
    /* Enable DMA channel1 */
    DMA_Channel_Enable(DMA_CH1);
}
// ========================================================================
// 函数名称：Adc_Init
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：AD初始化
// ========================================================================
void Adc_Init(void)
{
    ADC_InitType ADC_InitStructure;
    NVIC_InitType NVIC_InitStructure;
    
    /* ADC regular sequencer */
    ADC_InitStructure.MultiChEn      = ENABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_REGULAR_SWSTRRCH;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = ADC_REGULAR_LEN_3;
    ADC_Initializes(&ADC_InitStructure);
    /* ADC regular channel configuration */
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_09_PB0,ADC_REGULAR_NUMBER_1);//speed
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_04_PA3,ADC_REGULAR_NUMBER_2);//vbus
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_04_PA3,ADC_REGULAR_NUMBER_3);//vbus
    ADC_Channel_Sample_Time_Config(ADC_Channel_09_PB0, ADC_SAMP_TIME_13CYCLES5);
    ADC_Channel_Sample_Time_Config(ADC_Channel_04_PA3, ADC_SAMP_TIME_13CYCLES5);
    ADC_Channel_Sample_Time_Config(ADC_Channel_04_PA3, ADC_SAMP_TIME_13CYCLES5);

    /* Set injected sequencer length */
    ADC_Injected_Channels_Number_Config(ADC_INJECTED_LEN_3);
    /* ADC injected channel Configuration */
    ADC_Injected_Sequence_Conversion_Number_Config(ADC_Channel_01_PA0, ADC_INJECTED_NUMBER_2);
    ADC_Injected_Sequence_Conversion_Number_Config(ADC_Channel_02_PA1, ADC_INJECTED_NUMBER_3);
    ADC_Injected_Sequence_Conversion_Number_Config(ADC_Channel_03_PA2, ADC_INJECTED_NUMBER_4);				
    ADC_Channel_Sample_Time_Config(ADC_Channel_01_PA0, ADC_SAMP_TIME_13CYCLES5);
    ADC_Channel_Sample_Time_Config(ADC_Channel_02_PA1, ADC_SAMP_TIME_13CYCLES5);    
    ADC_Channel_Sample_Time_Config(ADC_Channel_03_PA2, ADC_SAMP_TIME_13CYCLES5);
    /* ADC injected external trigger configuration */
    ADC_Injected_Group_External_Trigger_Source_Config(ADC_EXT_TRIGCONV_INJECTED_T1_CC4);
    /* Enable automatic injected conversion start after regular one */
    ADC_Injected_Group_Autoconversion_Disable();


    /* Enable ADC */
    ADC_ON();
    /* Check ADC Ready */
    while(ADC_Flag_Status_Get(ADC_RD_FLAG, ADC_FLAG_JENDCA, ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC calibration */
    ADC_Calibration_Operation(ADC_CALIBRATION_ENABLE);
    /* Check the end of ADC calibration */
    while (ADC_Calibration_Operation(ADC_CALIBRATION_STS))
        ;

    /* Configure and enable ADC interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
    /* Enable JEOC interrupt */
    ADC_Interrupts_Enable(ADC_INT_JENDC);
    
    /* Enable ADC DMA */
    ADC_DMA_Transfer_Enable();
    /* Enable ADC external trigger */
    ADC_External_Trigger_Conversion_Config(ADC_EXTTRIGCONV_REGULAR_ENABLE);
	/* Enable ADC inj external trigger */
	ADC_External_Trigger_Conversion_Config(ADC_EXTTRIGCONV_INJECTED_ENABLE);
    
    /*Enable Dma*/
    DMA_AdcSequenceIntiall();
}





