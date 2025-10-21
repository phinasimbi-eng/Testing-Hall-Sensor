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
 * @file pwm.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "SystemDefine.h"
#include "UserParam.h"

void Pwm_Init(void);				// Pwm��ʼ��
void Pwm_II_Init(void);
void PwmDutySet_I(uint8_t num,int16_t duty1,int16_t duty2,int16_t duty3);// ����ռ�ձ�
void AllPwmShut(uint8_t num);  // ����ȫ��
void AllPwmOpen(uint8_t num);  // ����ȫ��
void SetPwmFreq(uint16_t Freq); // ����pwmƵ��
void Brake(uint8_t num);		// ɲ��
extern __IO uint16_t PWM_CcSet[3];

// ========================================================================
// �������ƣ�Pwm_Init
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������PWM��ʼ��
// ========================================================================
void Pwm_Init(void)
{
	TIM_TimeBaseInitType TIM1_TimeBaseStructure;
	OCInitType TIM1_OCInitStructure;
	TIM_BDTRInitType TIM1_BDTRInitStructure;
		
	uint16_t TimerPeriod = 0;

	TimerPeriod = (MAIN_FREQUENCY / (PWM_FREQUENCY1*2)) - 1;
	
	//Time Base configuration
	TIM_Reset(TIM1);
	TIM_Base_Struct_Initialize(&TIM1_TimeBaseStructure);
	TIM1_TimeBaseStructure.Prescaler = 0;
	TIM1_TimeBaseStructure.CntMode = TIM_CNT_MODE_CENTER_ALIGN1;// 01: \,irq flag only counter down
	TIM1_TimeBaseStructure.Period = TimerPeriod;
	TIM1_TimeBaseStructure.ClkDiv = TIM_CLK_DIV1;
	TIM1_TimeBaseStructure.RepetCnt = 0;
	
	TIM_Base_Initialize(TIM1, &TIM1_TimeBaseStructure);
	//Channel 1, 2,3 in PWM mode
	//TIM_InitOcStruct(&TIM1_OCInitStructure);
	TIM1_OCInitStructure.OcMode = TIM_OCMODE_PWM1;//Pos logic(when '<' is active,when '>' is inactive)
	TIM1_OCInitStructure.OutputState = TIM_OUTPUT_STATE_DISABLE; 
	TIM1_OCInitStructure.OutputNState = TIM_OUTPUT_NSTATE_DISABLE;                  
	TIM1_OCInitStructure.Pulse = (TimerPeriod>>1);
	TIM1_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;

	TIM1_OCInitStructure.OcNPolarity = TIM_OCN_POLARITY_HIGH; 

	TIM1_OCInitStructure.OcIdleState = TIM_OC_IDLE_STATE_RESET;
	TIM1_OCInitStructure.OcNIdleState = TIM_OC_IDLE_STATE_RESET;          
	TIM_Output_Channel1_Initialize(TIM1, &TIM1_OCInitStructure); 
	TIM_Output_Channel2_Initialize(TIM1, &TIM1_OCInitStructure);
	TIM_Output_Channel3_Initialize(TIM1, &TIM1_OCInitStructure);
	//Channel 4 Configuration in OC 
	TIM1_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
	TIM1_OCInitStructure.Pulse = TimerPeriod - 20;//3400;
	TIM_Output_Channel4_Initialize(TIM1, &TIM1_OCInitStructure);
	
	//Enables the TIM1 Preload on CC1,CC2,CC3,CC4 Register
	TIM_Output_Channel1_Preload_Set(TIM1, TIM_OC_PRELOAD_ENABLE);
	TIM_Output_Channel2_Preload_Set(TIM1, TIM_OC_PRELOAD_ENABLE);
	TIM_Output_Channel3_Preload_Set(TIM1, TIM_OC_PRELOAD_ENABLE);
    TIM_Output_Channel4_Preload_Set(TIM1,TIM_OC_PRELOAD_ENABLE);

	//Automatic Output enable, Break, dead time and lock configuration
	TIM1_BDTRInitStructure.OssrState = TIM_OSSR_STATE_ENABLE;
	TIM1_BDTRInitStructure.OssiState = TIM_OSSI_STATE_ENABLE;
	TIM1_BDTRInitStructure.LockLevel = TIM_LOCK_LEVEL_OFF; 
	TIM1_BDTRInitStructure.DeadTime = 64;
	TIM1_BDTRInitStructure.Break = TIM_BREAK_IN_DISABLE;//TIM_BREAK_IN_ENABLE;
	TIM1_BDTRInitStructure.BreakPolarity = TIM_BREAK_POLARITY_HIGH;
	TIM1_BDTRInitStructure.AutomaticOutput = TIM_AUTO_OUTPUT_ENABLE;
    TIM1_BDTRInitStructure.IomBreakEn = false;
	TIM_Break_And_Dead_Time_Set(TIM1, &TIM1_BDTRInitStructure);

	//TIM1 counter enable
	TIM_On(TIM1);
	TIM_PWM_Output_Enable(TIM1);
}
// ========================================================================
// �������ƣ�PwmDutySet_I
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������PWM����ռ�ձ�
// ========================================================================
void PwmDutySet_I(uint8_t num,int16_t duty1,int16_t duty2,int16_t duty3)
{
	if (num == 0)
	{
		TIM1->CCDAT1 = duty1; // U��ռ�ձ�
		TIM1->CCDAT2 = duty2; // V��ռ�ձ�
		TIM1->CCDAT3 = duty3; // W��ռ�ձ�
	}
	else if (num == 1)
	{
		TIM8->CCDAT1 = duty1; // U��ռ�ձ�
		TIM8->CCDAT2 = duty2; // V��ռ�ձ�
		TIM8->CCDAT3 = duty3; // W��ռ�ձ�
	}
}
// ========================================================================
// �������ƣ�AllPwmShut
// �����������
// �����������
// ��    �룺��
// ��    ������
// ��������������ȫ��
// ========================================================================
void AllPwmShut(uint8_t num)
{
    uint16_t tmp;
	if (num == 0)
	{
        tmp = TIM1->CCEN;
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1NEN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2NEN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3NEN));
        TIM1->CCEN = tmp;
	}
	else if (num == 1)
	{        
        tmp = TIM8->CCEN;
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1NEN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2NEN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3NEN));
        TIM8->CCEN = tmp;
	}
}
// ========================================================================
// �������ƣ�AllPwmOpen
// �����������
// �����������
// ��    �룺��
// ��    ������
// ��������������ȫ��
// ========================================================================
void AllPwmOpen(uint8_t num)
{
    uint16_t tmp;
	if (num == 0)
	{
        tmp = TIM1->CCEN;
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC1NEN));
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC2NEN)); 
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC3NEN)); 
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC1EN));
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC2EN)); 
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC3EN)); 
        TIM1->CCEN = tmp;
	}
	else if (num == 1)
	{
        tmp = TIM8->CCEN;
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC1NEN));
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC2NEN)); 
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC3NEN)); 
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC1EN));
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC2EN)); 
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC3EN)); 
        TIM8->CCEN = tmp;
	}
}
// ========================================================================
// �������ƣ�SetPwmFreq
// �����������
// �����������
// ��    �룺��
// ��    ������
// ��������������pwmƵ��
// ========================================================================
void SetPwmFreq(uint16_t Freq)
{
	uint16_t FreqTemp = 0;
	
	FreqTemp = ((uint16_t)MAIN_FREQUENCY >> 1) / Freq;
	
	TIM1->AR = FreqTemp;
	TIM1->CCDAT4 = (TIM1->AR - 30);
}
// ========================================================================
// �������ƣ�Brake
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������ɲ��(���Źرգ����Ŵ�)
// ========================================================================
void Brake(uint8_t num)
{
    uint16_t tmp;
	if (num == 0)
	{
        tmp = TIM1->CCEN;
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC1NEN));
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC2NEN)); 
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC3NEN)); 
        TIM1->CCEN = tmp;
	}
	else if (num == 1)
	{
        tmp = TIM8->CCEN;
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC1NEN));
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC2NEN)); 
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC3NEN)); 
        TIM8->CCEN = tmp;
	}
}

