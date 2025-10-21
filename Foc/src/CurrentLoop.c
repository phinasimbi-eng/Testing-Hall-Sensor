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

extern float CurDKp,CurDKi,CurQKp,CurQKi;
extern int32_t MCM_Sqrt( int32_t wInput );
// ========================================================================
// �������ƣ�CurLoopInit_c
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������������ִ�в�����ʼ������
// ========================================================================
void CurLoopInit_c(Motor_Obj *Obj)
{
    Obj->CurLoop.IdFilterTmp = 0;               // Id�˲��ݴ�
    Obj->CurLoop.IqFilterTmp = 0;               // Id�˲��ݴ�
    Obj->CurLoop.IdFilter = 0;                  // Id�˲�ֵ
    Obj->CurLoop.IqFilter = 0;                  // Id�˲�ֵ
	
    Obj->CurLoop.IdFedBak = 0;                  // Id����
    Obj->CurLoop.IqFedBak = 0;                  // Iq����

		Obj->CurLoop.IdSumTemp = 0;                 // Id�����ݴ�
    Obj->CurLoop.IqSumTemp = 0;                 // Iq�����ݴ�
		
    Obj->CurLoop.PiIdSum = 0;                   // Id����
    Obj->CurLoop.PiIqSum = 0;                   // Id����
    Obj->CurLoop.PiIdOut = 0;                   // D��Pi���
		Obj->CurLoop.PiIqOut = 0;                   // Q��Pi���

		Obj->CurLoop.UdOut_Q15 = 0;                 // Ud���
		Obj->CurLoop.UqOut_Q15 = 0;                 // Uq���

		Obj->DataObj.IqObj = 0;
		Obj->DataObj.IdObj = 0;	
		
		Obj->CurLoop.Us2 = 0;
		Obj->CurLoop.Ud2 = 0;
}
// ========================================================================
// �������ƣ�CurLoop_Isr_c
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������D��Q �������
// ========================================================================
void CurLoop_Isr_c(Motor_Obj *Obj)
{
	int32_t IdTmp_error,IqTmp_error;

	IdTmp_error = Obj->DataObj.IdObj - Obj->CurLoop.IdFedBak;
	Obj->CurLoop.PiIdSum = (Obj->CurLoop.UdOut_Q15 == Obj->CurLoop.PiIdOut)?(_IQmpy(Obj->MotorBaseObj.Fun_DKi_Q15, IdTmp_error) + Obj->CurLoop.IdSumTemp) : Obj->CurLoop.IdSumTemp;
	Obj->CurLoop.IdSumTemp = Obj->CurLoop.PiIdSum;
	Obj->CurLoop.PiIdOut = _IQmpy(Obj->MotorBaseObj.Fun_DKp_Q13, IdTmp_error) + Obj->CurLoop.PiIdSum;
	Obj->CurLoop.UdOut_Q15 = LLimit(Obj->CurLoop.PiIdOut, -Obj->MotorBaseObj.Fun_DOutMax_Q15, Obj->MotorBaseObj.Fun_DOutMax_Q15);
	
	IqTmp_error = Obj->DataObj.IqObj - Obj->CurLoop.IqFedBak;
	Obj->CurLoop.PiIqSum = (Obj->CurLoop.UqOut_Q15 == Obj->CurLoop.PiIqOut)?(_IQmpy(Obj->MotorBaseObj.Fun_QKi_Q15, IqTmp_error) + Obj->CurLoop.IqSumTemp) : Obj->CurLoop.IqSumTemp;
	Obj->CurLoop.IqSumTemp = Obj->CurLoop.PiIqSum;
	Obj->CurLoop.PiIqOut = _IQmpy(Obj->MotorBaseObj.Fun_QKp_Q13, IqTmp_error) + Obj->CurLoop.PiIqSum;	
	Obj->CurLoop.UqOut_Q15 = LLimit(Obj->CurLoop.PiIqOut, -Obj->MotorBaseObj.Fun_QOutMax_Q15, Obj->MotorBaseObj.Fun_QOutMax_Q15);
}
