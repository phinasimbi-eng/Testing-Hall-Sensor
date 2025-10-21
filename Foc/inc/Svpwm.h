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
 * @file svpwm.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _SVPWM_H_
#define _SVPWM_H_

#include "stdint.h"
#define SQRT_3__3  18918l	 //sqrt(3)/3 <<15
#define SQRT_3  1774			// sqrt(3) << 10

typedef struct _SVPWM_OBJ_
{
	int16_t Ua;           // 计算扇区第一个相关量
	int16_t Ub;           // 计算扇区第二个相关量
	int16_t Uc;           // 计算扇区第三个相关量
	
	int16_t SinResult;    // sin值
	int16_t CosResult;    // Cos值
	int32_t U_Alpha;
	int32_t U_Beta;
	int16_t Sector;       // 扇区值
	int32_t DutyA;        // A相占空比
	int32_t DutyB;        // B相占空比
	int32_t DutyC;        // C相占空比		
	int32_t Half_Duty;	
} Svpwm_Obj;




#endif 
