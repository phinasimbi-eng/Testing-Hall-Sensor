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
 * @file brushless.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef BRUSHLESS_H_
#define BRUSHLESS_H_

#include "stdint.h"

//#define TORQUE_CONTROL

typedef struct _BRUSHLESS_OBJ_
{
	int16_t EleAng_Q15;    
	int32_t I_Alpha;
	int32_t I_Beta;
	int16_t Omeg_Foc;
	uint16_t IqUpCnt;
	int16_t IqTarget;
	int16_t IqTargetTorqueDef;
    int16_t IqTorqStep;
    
    //open angle
	int32_t StartupRamp;
	int32_t Final_Position;
	int16_t Open_DelTHeta;
	int16_t Open_Angle; 
    //smo need variable
    uint8_t CloseLoopFlag;
	int16_t Angle_Err;
} Brushless_Obj;

typedef struct _DEAD_COMP_OBJ_
{
    int16_t Iq_filterd;
    int32_t Iq_filtertemp;
    int16_t Id_filterd;
    int32_t Id_filtertemp;
    
    int16_t I_alpha;
    int16_t I_beta;
    
    int16_t I_A;
    int16_t I_B;
    int16_t I_C;
    int16_t Comp_count;
}DeadComp_Obj;

#endif /* BRUSHLESS_H_ */
