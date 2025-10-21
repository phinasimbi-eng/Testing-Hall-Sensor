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
 * @file errdeal.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "MotorDrive.h"
// ========================================================================
// 函数名称：ErrDealInit_I
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：ErrDeal_Init初始化
// ========================================================================
void ErrDealInit(Motor_Obj *Obj)
{
	Obj->Flag.Arr[0] = 0;
	Obj->Flag.Arr[1] = 0;
	
}
// ========================================================================
// 函数名称：ErrDeal_Isr
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：ErrDeal_Isr处理函数
// ========================================================================
void ErrDeal_Isr(Motor_Obj *Obj)
{
	if (Obj->Flag.Bits.SystemReady == 1)
	{
		if ((Obj->DataObj.Ia_PhyMax > Obj->MotorBaseObj.OverCurrent) || (Obj->DataObj.Ib_PhyMax > Obj->MotorBaseObj.OverCurrent) \
			|| (Obj->DataObj.Ic_PhyMax > Obj->MotorBaseObj.OverCurrent))
		{
			Obj->DataObj.CurrentCnt++;
			if (Obj->DataObj.CurrentCnt > 0)
			{
				Obj->Flag.Bits.MotorStarted = 0;
				AllPwmShut(0);
				Obj->DataObj.CurrentCnt = 0;
				Obj->Flag.Bits.SwOverCurrent = 1;
        LEDON_Control(LED_CURRENT_ERR);					
			}
		}
		else
		{
			Obj->DataObj.CurrentCnt = 0;
			if (Obj->Flag.Bits.SwOverCurrent == 1)
			{
				if (Obj->MotorBaseObj.AllowReset == 1)
				{
					Obj->Flag.Bits.SwOverCurrent = 0;
				}
			}
			else
			{
				Obj->Flag.Bits.SwOverCurrent = 0;
			}
		}
		if (Obj->DataObj.Udc_Real > Obj->MotorBaseObj.OverVoltage)
		{
			Obj->DataObj.OverVoltageCnt++;
			if (Obj->DataObj.OverVoltageCnt > 0)
			{
				Obj->DataObj.OverVoltageCnt = 0;
				Obj->Flag.Bits.OverVolatage = 1;
        LEDON_Control(LED_VOLTAGE_ERR);				
			}
		}
		else
		{
			Obj->DataObj.OverVoltageCnt = 0;
			if (Obj->Flag.Bits.OverVolatage == 1)
			{
				if (Obj->MotorBaseObj.AllowReset == 1)
				{
					Obj->Flag.Bits.OverVolatage = 0;								
				}
			}
			else
			{
				Obj->Flag.Bits.OverVolatage = 0;
			}
		}
		if ((Obj->DataObj.Udc_Real < Obj->MotorBaseObj.LackVoltage) && (Obj->Flag.Bits.ElectUpFlag == 1))
		{
			Obj->DataObj.LackVoltageCnt++;
			if (Obj->DataObj.LackVoltageCnt > 0)
			{
				Obj->DataObj.LackVoltageCnt = 0;
				Obj->Flag.Bits.LackVolatage = 1;
        LEDON_Control(LED_VOLTAGE_ERR);				
			}
		}
		else
		{
			Obj->DataObj.LackVoltageCnt = 0;
			if (Obj->Flag.Bits.LackVolatage == 1)
			{
				if (Obj->MotorBaseObj.AllowReset == 1)
				{
					Obj->Flag.Bits.LackVolatage = 0;
				}
			}
			else
			{
				Obj->Flag.Bits.LackVolatage = 0;
			}
		}
	}
	else
	{;}
}

