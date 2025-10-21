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
 * @file currentloop.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _CURRENTLOOP_H_
#define _CURRENTLOOP_H_

#include "stdint.h"

typedef struct _CURLOOP_OBJ_
{
    float IdFilterTmp;               // Id滤波暂存
    float IqFilterTmp;               // Id滤波暂存
    float IdFilter;               	 // Id滤波值
    float IqFilter;               	 // Id滤波值
		
    int32_t ThetaComp; 								 // 控制延迟补偿角
    int32_t SinResult;
    int32_t CosResult;
    int32_t UdOutComp;                 // Ud输出补偿后值
    int32_t UqOutComp;                 // Uq输出补偿后值		
	
    int16_t IdFedBak;                  // Id反馈
    int16_t IqFedBak;                  // Iq反馈

    int32_t IdSumTemp;                 // Id积分暂存
    int32_t IqSumTemp;                 // Iq积分暂存
	
    int16_t PiIdSum;                   // Id积分
    int16_t PiIqSum;                   // Id积分
    int16_t PiIdOut;                   // D环Pi输出
    int16_t PiIqOut;                   // Q环Pi输出

    int32_t UdOut_Q15;                 // Ud输出
    int32_t UqOut_Q15;                 // Uq输出
	int32_t Us2;
	int32_t Ud2;	
} CurLoop_Obj;




#endif /* CURRENTLOOP_H_ */
