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
 * @file brushlesspictrl.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "MotorDrive.h"
#include "math.h"

void CurLoop_Isr(Motor_Obj *Obj);
void CurLoop_Isr_c(Motor_Obj *Obj);
// ========================================================================
// 函数名称：BrushlessInit_I
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：BrushlessInit_I初始化
// ========================================================================
void BrushlessInit(Motor_Obj *Obj)
{
    Obj->BrushlessObj.EleAng_Q15 = 0;
    Obj->BrushlessObj.Omeg_Foc = 0;
    Obj->BrushlessObj.I_Alpha = 0;
    Obj->BrushlessObj.I_Beta = 0;
    Obj->BrushlessObj.IqUpCnt = 0;
    Obj->BrushlessObj.IqTarget = 0;
    
    Obj->BrushlessObj.Open_Angle = 0; 
    Obj->BrushlessObj.Open_DelTHeta = 0;
    Obj->BrushlessObj.StartupRamp = 0;
    Obj->BrushlessObj.Final_Position = 20*65536/Obj->Plate.Pwm_freq*256*256;	//858880
}
// ========================================================================
// 函数名称：BrushlessPiCtrl
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：电流环
// ========================================================================
void BrushlessPiCtrl(uint8_t num,Motor_Obj *Obj)
{
	CurLoop_Isr_c(Obj);		
}
