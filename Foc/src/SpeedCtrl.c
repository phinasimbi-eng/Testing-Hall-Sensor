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
 * @file speedctrl.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "MotorDrive.h"
#include "math.h"

extern float SpeedKp,SpeedKi;
// ========================================================================
// 函数名称：SpeedCtrlInit_I
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：SpeedCtrlInit_I初始化
// ========================================================================
void SpeedCtrlInit(uint8_t num,Motor_Obj *Obj)
{
	Obj->SpeedObj.UsDifSumBuff = 0;
	Obj->SpeedObj.UsDifOut = 0;
	Obj->SpeedObj.UsDifErr = 0;
	Obj->SpeedObj.UsDifSum = 0;
}
// ========================================================================
// 函数名称：Speed_air_2ms
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：功率控制
// ========================================================================
void Speed_air_2ms(uint8_t num,Motor_Obj *Obj)
{
    int32_t Up = 0;
    
    if (Obj->SpeedObj.target_calc - Obj->SpeedObj.Speed_Target > Obj->SpeedObj.step_target)
    {
        Obj->SpeedObj.target_calc -= Obj->SpeedObj.step_target;
    }
    else if (Obj->SpeedObj.Speed_Target - Obj->SpeedObj.target_calc > Obj->SpeedObj.step_target)
    {
        Obj->SpeedObj.target_calc += Obj->SpeedObj.step_target;
    }
    else
    {
        Obj->SpeedObj.target_calc = Obj->SpeedObj.Speed_Target;
    }
		
    int32_t maxvalue = 0;
    Obj->SpeedObj.UsDifErr = Obj->SpeedObj.target_calc - Obj->DataObj.MotorSpeed;
    Up = (Obj->SpeedObj.UsDifErr * Obj->MotorBaseObj.Fun_SpdKp) >> 10;
    Obj->SpeedObj.UsDifSum += (Obj->SpeedObj.UsDifErr * Obj->MotorBaseObj.Fun_SpdKi);
    maxvalue = ((int32_t)Obj->MotorBaseObj.Fun_SpdOutMax << 15);
    Obj->SpeedObj.UsDifSum = LLimit(Obj->SpeedObj.UsDifSum, -maxvalue, maxvalue);
    Obj->SpeedObj.UsDifOut = LLimit((Up + (Obj->SpeedObj.UsDifSum >> 15)), -Obj->MotorBaseObj.Fun_SpdOutMax, Obj->MotorBaseObj.Fun_SpdOutMax);
    
    if (Obj->Flag.Bits.MotorStarted == 0)
    {
        Obj->SpeedObj.target_calc = 0;
        Obj->SpeedObj.UsDifSum = 0;
        Obj->CurLoop.IqFilterTmp = 0;
    }
    else
    {} 
#ifndef TORQUE_CONTROL				
    Obj->DataObj.IqObj = Obj->SpeedObj.UsDifOut;
    Obj->DataObj.IdObj = 0;			
#endif				
}


