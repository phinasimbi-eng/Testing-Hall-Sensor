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
 * @file IdIqfedbak.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#include "MotorDrive.h"

extern Motor_Obj motor_I[2];
extern System_Obj SystemObj[2];
extern __IO uint16_t ADC_InjectedConvertedValueTab[4];
extern __IO uint16_t ADC_RegularConvertedValueTab[8];
extern int32_t  IQsat( int32_t Uint,int32_t  U_max, int32_t U_min);
extern uint16_t* address_adc_value[2];
extern void ErrDeal_Isr(Motor_Obj *Obj);       // 故障处理函数
// ========================================================================
// 函数名称：DataDealInit_c
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：参数初始化函数
// ========================================================================
void DataDealInit_c(Motor_Obj *Obj)
{
		Obj->DataObj.ZeroCal_ASum = 0;      // A相零漂均值累积
		Obj->DataObj.ZeroCal_BSum = 0;      // B相零漂均值累积
		Obj->DataObj.ZeroCal_CSum = 0;      // C相零漂均值累积

		Obj->DataObj.ZeroCal_ArrASum = 0;   // 滑窗滤波数组和
		Obj->DataObj.ZeroCal_ArrBSum = 0;   // 滑窗滤波数组和
		Obj->DataObj.ZeroCal_ArrCSum = 0;   // 滑窗滤波数组和

		Obj->DataObj.Ia_MaxBuff = 0;        // A相最大值暂存器
		Obj->DataObj.Ib_MaxBuff = 0;        // B相最大值暂存器
		Obj->DataObj.Ic_MaxBuff = 0;        // C相最大值暂存器
		
		Obj->DataObj.Ia_Q15 = 0;            // A相电流Q15
		Obj->DataObj.Ib_Q15 = 0;            // B相电流Q15
		Obj->DataObj.Ic_Q15 = 0;            // C相电流Q15
		Obj->DataObj.Ia_Phy = 0;            // A相电流物理值
		Obj->DataObj.Ib_Phy = 0;            // B相电流物理值
		Obj->DataObj.Ic_Phy = 0;            // C相电流物理值

		Obj->DataObj.Ia_PhyMax = 0;         // A相电流最大值
		Obj->DataObj.Ib_PhyMax = 0;         // B相电流最大值
		Obj->DataObj.Ic_PhyMax = 0;         // C相电流最大值

		Obj->DataObj.MotorSpeed = 0;        // 电机转速
				
		Obj->DataObj.PhaseAVolZeroRef = 0; // A相电压Q12
		Obj->DataObj.PhaseBVolZeroRef = 0; // B相电压Q12
		Obj->DataObj.PhaseCVolZeroRef = 0; // C相电压Q12		
}
// ========================================================================
// 函数名称：IzeroReCalc_c
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：零电流参考值计算
// ========================================================================
void IzeroReCalc_c(Motor_Obj *Obj,SystemInterface_Obj *InfObj)
{
	static uint16_t ADC_PhaseA_Curr[FilterNum];
	static uint16_t ADC_PhaseB_Curr[FilterNum];
	static uint16_t ADC_PhaseC_Curr[FilterNum];
	static uint8_t i = 0;	
	static uint8_t j = 0;	
	static uint8_t flag = 0;	
	
	ADC_PhaseA_Curr[i] = ADC_InjectedConvertedValueTab[0];
	ADC_PhaseB_Curr[i] = ADC_InjectedConvertedValueTab[1];
	ADC_PhaseC_Curr[i] = ADC_InjectedConvertedValueTab[2];

	i++;
	if(i>=FilterNum)
	i=0;

	if(j<=CalcuNum)
	{
		j++;
		flag=1;
	}
	else
		flag=0;
	
	if(Obj->Flag.Bits.MotorStarted == 0 && flag == 1)
	{
		uint32_t sum_U = 0;
		uint32_t sum_V = 0;	
		uint32_t sum_W = 0;		
		uint8_t i;
		for(i=0; i < FilterNum; i++)
		{
			sum_U += ADC_PhaseA_Curr[i];
			sum_V += ADC_PhaseB_Curr[i];
			sum_W += ADC_PhaseC_Curr[i];
		}
		Obj->DataObj.Ia_ZeroRef = _IQ12toIQ(sum_U /FilterNum);
		Obj->DataObj.Ib_ZeroRef = _IQ12toIQ(sum_V /FilterNum);
		Obj->DataObj.Ic_ZeroRef = _IQ12toIQ(sum_W /FilterNum);		
	}
	
}
// ========================================================================
// 函数名称：CurrentQ15Calcu_c
// 输入参数：无                                                                                                            
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：采集的相电流处理
// ========================================================================
void CurrentQ15Calcu_c(Motor_Obj *Obj,SystemInterface_Obj *InfObj)
{	
	Obj->DataObj.Ia_Q15 = ((_IQ12toIQ(ADC_InjectedConvertedValueTab[0]) - Obj->DataObj.Ia_ZeroRef));
	Obj->DataObj.Ib_Q15 = ((_IQ12toIQ(ADC_InjectedConvertedValueTab[1]) - Obj->DataObj.Ib_ZeroRef));
	Obj->DataObj.Ic_Q15 = ((_IQ12toIQ(ADC_InjectedConvertedValueTab[2]) - Obj->DataObj.Ic_ZeroRef));
	
	Obj->DataObj.Ia_PhyMax = _IQabs(Obj->DataObj.Ia_Q15);	//限流保护
	Obj->DataObj.Ib_PhyMax = _IQabs(Obj->DataObj.Ib_Q15);
	
	ErrDeal_Isr(&motor_I[0]);
}
// ========================================================================
// 函数名称：ElectUpDected_c
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：上电缓冲检测函数
// ========================================================================
void ElectUpDected_c(Motor_Obj *Obj)
{	
	Obj->DataObj.Udc_Q15 = (ADC_RegularConvertedValueTab[1] << 3);
	Obj->DataObj.Udc_Real = ((uint32_t)Obj->DataObj.Udc_Q15 * Obj->MotorBaseObj.VolatageBase) >> 15;	
	if(Obj->DataObj.Udc_Q15 > 0)
	{
		Obj->Flag.Bits.SystemReady = 1;		
	}
	else
	{
		Obj->Flag.Bits.SystemReady = 0;		
	}
	if(Obj->DataObj.Udc_Real < Obj->MotorBaseObj.OverVoltage &&
		 Obj->DataObj.Udc_Real > Obj->MotorBaseObj.LackVoltage)
	{
		Obj->Flag.Bits.ElectUpFlag = 1;		
	}	
}
// ========================================================================
// 函数名称：GetHallLine_c
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：霍尔相序检测函数
// ========================================================================
int8_t GetHallLine_c(HallCalc_t * pHallCalc,int8_t HallIo)
{
    uint8_t i = 0;
    pHallCalc->HallIo = HallIo;

    for(;i<6;i++)
    {
				if(pHallCalc->HallIo == 0)
				{
						break;
				}
        if(pHallCalc->HallIo == pHallCalc->HallFixLineSeq[i])
        {
            break;
        }
    }
		if(i >= 6) 
			return 1;
		else 
			return 0;
}
