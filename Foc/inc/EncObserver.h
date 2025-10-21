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
 * @file EncObserver.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#ifndef FOC_ENCOBSERVER_H
#define FOC_ENCOBSERVER_H

#include "stdint.h"
#include "stdbool.h"

typedef struct EncObserver_t
{
    //extern static param   
    int16_t PulsPerPeriod;
    int32_t Pwm;
    int8_t Pols;
    //angle and speed
    int16_t Angle;
    int32_t CalSpeedRpm;
    int32_t CalSpeedHz;
    int16_t initial;
    
    //angle and speed cnt
    int32_t CalAngleCnt;
    int32_t RuningPulsCnt; 
    int32_t CalAngleCntBak;
    int32_t RuningPulsCntBak;
    int32_t CalAngleExCnt;
    int32_t RuningPulsExCnt; 
    
    //run param
    int16_t EncodePuls;
    int16_t PreEncodePuls;
    //private
    int32_t MultC;
    //end
    uint8_t end;
}EncObserver_t;

void EncIntial(struct EncObserver_t * pObserver);
int16_t EncAngleCalc(struct EncObserver_t *pObserver,uint16_t GetEnc);
int16_t EncSpeedGet(struct EncObserver_t *pObserver);
void EndGetPosition(struct EncObserver_t *pObserver,int32_t *CalAngleExCnt,int32_t *RuningPulsExCnt);
#endif

