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
 * @file startstop.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "MotorDrive.h"
#include "math.h"

static void SysTemDect(Motor_Obj *Obj);
static void RunFlow(Motor_Obj *Obj);
static void MotorControl(Motor_Obj *Obj,uint8_t num);
// ========================================================================
// �������ƣ�StartStop_Isr
// �����������
// �����������
// ��    �룺��
// ��    ������
// ���������������ͣ����
// ========================================================================
void StartStop_Isr(Motor_Obj *Obj,uint8_t num)
{
	SysTemDect(Obj); 
	RunFlow(Obj);
	MotorControl(Obj,num);
}
// ========================================================================
// �������ƣ�SysTemDect
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������ϵͳ�����ж�
// ========================================================================

void SysTemDect(Motor_Obj *Obj)
{
	if ((Obj->Flag.Bits.SwOverCurrent == 0) && (Obj->Flag.Bits.HwTotalErr == 0) \
			&& (Obj->Flag.Bits.HwOverCurrent == 0) && (Obj->Flag.Bits.LackVolatage == 0) \
			&& (Obj->Flag.Bits.IZeroFlag == 0) \
			&& (Obj->Flag.Bits.ElectUpFlag == 1) && (Obj->SoftStartStopCtrl == 1) \
			&& (Obj->Flag.Bits.SystemReady == 1) && (Obj->Flag.Bits.HighTempFlag == 0) \
			&& (Obj->Flag.Bits.LowTempFlag == 0))
	{
		Obj->Flag.Bits.AllowStart = 1;
	}
	else
	{
		Obj->Flag.Bits.AllowStart = 0;
	}
}
// ========================================================================
// �������ƣ�RunFlow
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������״̬����
// ========================================================================
void RunFlow(Motor_Obj *Obj)
{
	if (Obj->MotorState == STOP_STATE)
	{
		if (Obj->Flag.Bits.AllowStart == 1)
		{
			Obj->MotorState = RUN_STATE;
		}
		if (Obj->Flag.Bits.BrakeFlag == 1)
		{
			Obj->MotorState = SHORTPHASE_STATE;
		}
	}
	if (Obj->MotorState == RUN_STATE)
	{
		if (Obj->Flag.Bits.AllowStart == 0)
		{
			Obj->MotorState = STOP_STATE;
		}
		if (Obj->Flag.Bits.BrakeFlag == 1)
		{
			Obj->MotorState = SHORTPHASE_STATE;
		}
	}
	if (Obj->MotorState == SHORTPHASE_STATE)
	{
		if (Obj->Flag.Bits.BrakeFlag == 0)
		{
			if (Obj->Flag.Bits.AllowStart == 0)
			{
				Obj->MotorState = STOP_STATE;
			}
			else
			{
				Obj->MotorState = RUN_STATE;
			}
		}
	}
}
// ========================================================================
// �������ƣ�MotorControl
// �����������
// �����������
// ��    �룺��
// ��    ������
// �����������������
// ========================================================================
void MotorControl(Motor_Obj *Obj,uint8_t num)
{
	if (Obj->MotorState == RUN_STATE)
	{
		Obj->Flag.Bits.MotorStarted = 1;
		AllPwmOpen(num);
	}
	else if (Obj->MotorState == SHORTPHASE_STATE)
	{
		Obj->Flag.Bits.MotorStarted = 0;
		Brake(num);
	}
	else if(Obj->MotorState == STOP_STATE)
	{
		Obj->Flag.Bits.MotorStarted = 0;
		AllPwmShut(num);
	}
	else
	{;} 
}




