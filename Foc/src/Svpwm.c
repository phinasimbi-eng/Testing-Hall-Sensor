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

// ========================================================================
// �������ƣ�SvpwmInit_c
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������ʸ������λ�ò�����ʼ������
// ========================================================================
void SvpwmInit_c(Motor_Obj *Obj,SystemInterface_Obj *InfObj)
{
	Obj->Svpwm.Ua = 0;           // ����������һ�������
	Obj->Svpwm.Ub = 0;           // ���������ڶ��������
	Obj->Svpwm.Uc = 0;           // �������������������

	Obj->Svpwm.SinResult = 0;    // sinֵ
	Obj->Svpwm.CosResult = 0;    // Cosֵ
	Obj->Svpwm.U_Alpha = 0;
	Obj->Svpwm.U_Beta = 0;
	Obj->Svpwm.Sector = 0;       // ����ֵ
	Obj->Svpwm.DutyA = HALFDUTY_1;        // A��ռ�ձ�
	Obj->Svpwm.DutyB = HALFDUTY_1;        // B��ռ�ձ�
	Obj->Svpwm.DutyC = HALFDUTY_1;        // C��ռ�ձ�
	Obj->Svpwm.Half_Duty = HALFDUTY_1;
}
// ========================================================================
// �������ƣ�SvpwmNew_Isr_c
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������ʸ������
// ========================================================================
void SvpwmNew_Isr_c(uint8_t num,Motor_Obj *Obj,int32_t UdOut_Q15,int32_t UqOut_Q15)
{
	int32_t DutyA,DutyB,DutyC;

	Obj->Svpwm.U_Alpha = ((Obj->Svpwm.CosResult * Obj->CurLoop.UdOut_Q15) - (Obj->Svpwm.SinResult * Obj->CurLoop.UqOut_Q15)) >> 15;
	Obj->Svpwm.U_Beta = ((Obj->Svpwm.SinResult * Obj->CurLoop.UdOut_Q15) + (Obj->Svpwm.CosResult * Obj->CurLoop.UqOut_Q15)) >> 15;
	
  Obj->Svpwm.Ua= Obj->Svpwm.U_Beta;
	Obj->Svpwm.Ub= _IQdiv2(Obj->Svpwm.U_Beta) + _IQmpy(28377,Obj->Svpwm.U_Alpha);
  Obj->Svpwm.Uc= Obj->Svpwm.Ub - Obj->Svpwm.Ua;

	Obj->Svpwm.Sector=3;
	Obj->Svpwm.Sector=(Obj->Svpwm.Ub> 0)?(Obj->Svpwm.Sector-1):Obj->Svpwm.Sector;
	Obj->Svpwm.Sector=(Obj->Svpwm.Uc> 0)?(Obj->Svpwm.Sector-1):Obj->Svpwm.Sector;
	Obj->Svpwm.Sector=(Obj->Svpwm.Ua< 0)?(7-Obj->Svpwm.Sector):Obj->Svpwm.Sector;

	 if  (Obj->Svpwm.Sector==1 || Obj->Svpwm.Sector==4)
      {  
				DutyA= Obj->Svpwm.Ub;
      	DutyB= Obj->Svpwm.Ua-Obj->Svpwm.Uc;
      	DutyC=-Obj->Svpwm.Ub;
      }

   else if(Obj->Svpwm.Sector==2 || Obj->Svpwm.Sector==5)
      {  
				DutyA= Obj->Svpwm.Uc+Obj->Svpwm.Ub;
      	DutyB= Obj->Svpwm.Ua;
      	DutyC=-Obj->Svpwm.Ua;
      }

    else
      {   
				DutyA= Obj->Svpwm.Uc;
      	DutyB=-Obj->Svpwm.Uc;
      	DutyC=-(Obj->Svpwm.Ua+Obj->Svpwm.Ub);
      }
	Obj->Svpwm.DutyA = _IQmpy(Obj->Svpwm.Half_Duty,DutyA)+ Obj->Svpwm.Half_Duty;		
	Obj->Svpwm.DutyB = _IQmpy(Obj->Svpwm.Half_Duty,DutyB)+ Obj->Svpwm.Half_Duty;		
	Obj->Svpwm.DutyC = _IQmpy(Obj->Svpwm.Half_Duty,DutyC)+ Obj->Svpwm.Half_Duty;				
}

