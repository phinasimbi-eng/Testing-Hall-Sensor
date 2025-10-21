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
extern void Observer_Init(Motor_Obj *Obj);
extern int16_t	Observer_Run(uint8_t num,Motor_Obj *Obj);
// ========================================================================
// 函数名称：IdIqFbCalcu_c
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：IdIq反馈计算
// ========================================================================
void IdIqFbCalcu_c(Motor_Obj *Obj)
{
	int32_t Temp32_dq = 0;
	Obj->BrushlessObj.I_Alpha = Obj->DataObj.Ia_Q15;	
	Obj->BrushlessObj.I_Beta = ((int32_t)SQRT_3__3 * (Obj->DataObj.Ia_Q15 + (Obj->DataObj.Ib_Q15 << 1))) >> 15;

  Obj->Svpwm.SinResult = Sinlt(Obj->BrushlessObj.EleAng_Q15);
	Obj->Svpwm.CosResult = Coslt(Obj->BrushlessObj.EleAng_Q15);
	
	Temp32_dq = (-Obj->BrushlessObj.I_Alpha * Obj->Svpwm.SinResult + Obj->BrushlessObj.I_Beta * Obj->Svpwm.CosResult)>>15;   //Iq
	Obj->CurLoop.IqFedBak = (Temp32_dq > 32767)?32767:((Temp32_dq < -32768)?-32768:Temp32_dq);
	
	Temp32_dq= (Obj->BrushlessObj.I_Alpha * Obj->Svpwm.CosResult + Obj->BrushlessObj.I_Beta * Obj->Svpwm.SinResult)>>15;   //Id
	Obj->CurLoop.IdFedBak = (Temp32_dq > 32767)?32767:((Temp32_dq < -32768)?-32768:Temp32_dq);
}
