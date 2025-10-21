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
 * @file datadeal.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _DATADEAL_H_
#define _DATADEAL_H_

#include "stdint.h"
#define IMAXCLCUCYC 100

typedef struct _DATA_OBJ_
{
    int16_t ZeroCal_ReadyCnt;  // 零漂稳定计数
    int32_t ZeroCal_ASum;      // A相零漂均值累积
    int32_t ZeroCal_BSum;      // B相零漂均值累积
    int32_t ZeroCal_CSum;      // C相零漂均值累积

    int16_t ZeroCal_Cnt;       // 零漂计算计数
    int32_t ZeroCal_ArrASum;   // 滑窗滤波数组和
    int32_t ZeroCal_ArrBSum;   // 滑窗滤波数组和
    int32_t ZeroCal_ArrCSum;   // 滑窗滤波数组和

    int16_t ZeroCal_ArrIndex;  // 滑窗滤波计数
    int16_t ZeroCal_ArrA[16];  // 滑窗滤波数组A
    int16_t ZeroCal_ArrB[16];  // 滑窗滤波数组B
    int16_t ZeroCal_ArrC[16];  // 滑窗滤波数组C

    int16_t ZeroRecal_Cnt;     // 零漂重算计数
    int16_t Ia_ZeroRef;        // A相零漂参考值
    int16_t Ib_ZeroRef;        // B相零漂参考值
    int16_t Ic_ZeroRef;        // C相零漂参考值

    int16_t IMaxCalcuCnt;      // 电流最大值计数器
    int16_t Ia_MaxBuff;        // A相最大值暂存器
    int16_t Ib_MaxBuff;        // B相最大值暂存器
    int16_t Ic_MaxBuff;        // C相最大值暂存器

    int16_t Udc_Q15;           // 母线电压Q15
    int16_t Udc_Real;          // 母线电压实际值
    int32_t Ia_Q15;            // A相电流Q15
    int32_t Ib_Q15;            // B相电流Q15
    int32_t Ic_Q15;            // C相电流Q15
//    int16_t Ia_Phy;            // A相电流物理值
//    int16_t Ib_Phy;            // B相电流物理值
//    int16_t Ic_Phy;            // C相电流物理值
    float Ia_Phy;            // A相电流物理值
    float Ib_Phy;            // B相电流物理值
    float Ic_Phy;            // C相电流物理值

    int16_t Ia_PhyMax;         // A相电流最大值
    int16_t Ib_PhyMax;         // B相电流最大值
    int16_t Ic_PhyMax;         // C相电流最大值

    int32_t MotorSpeed;        // 电机转速

    int16_t IdObj;             // Id目标值
    int16_t IqObj;             // Iq目标值
				
	int16_t  PhaseAVolZeroRef; // A相电压Q12
	int16_t  PhaseBVolZeroRef; // B相电压Q12
	int16_t  PhaseCVolZeroRef; // C相电压Q12
				
                
		
	uint16_t CurrentCnt;
	uint16_t OverVoltageCnt;
	uint16_t LackVoltageCnt;
	
	uint16_t HwTotalErrCnt;	
} Data_Obj;


#endif 
