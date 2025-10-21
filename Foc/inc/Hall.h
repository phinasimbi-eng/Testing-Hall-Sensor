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
 * @file Hall.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#ifndef _HALL_H_
#define _HALL_H_


#include "stdint.h"

#define HALL_STUDY_LEN 6
typedef struct{
    int8_t Enable;
    int16_t CurrentValue;
    int16_t ForceAngInc;
    int8_t HallTmp;
    int8_t HallBuf[HALL_STUDY_LEN];
    int16_t AngBuf[HALL_STUDY_LEN];
    int8_t HallTmpPre;
    int16_t Halli;
    int16_t Filter;
    int16_t HallAngle;
}HallStudy_t;

int16_t HallAngleStudy(HallStudy_t * pHallStudy,int16_t ForceAngInc/*=0*/);



typedef struct 
{
    int16_t Pwm_freq;		        //载频
    int8_t Pols;                    //极对数
    uint8_t HallFixLineSeq[6];   //Hall线序
    int16_t HallFixAngle[6];     //Hall角度
    int8_t HallIo;                 //Hall状态
    int8_t Index;  
    int8_t IndexOld;   
    int16_t HallAngle;
    int16_t HallAngleOffset;
    int16_t PosNegCnt;
    int32_t AngleCalTickCnt;//有符号
    int32_t AngleCalTickCntPre;//无符号
    int32_t AngleCalTickInc;
    int32_t AngleBaseCntTotal;
    int32_t HallSpeed;
    int32_t HallSpeedAvg;
    #define MaxSpdBuf 8
    int32_t SpeedBuf[MaxSpdBuf];
    uint16_t Spbi;
    int32_t Delay;
}HallCalc_t;

void HallIntial(HallCalc_t * pHallCalc);
int16_t HallAngleCalc(HallCalc_t * pHallCalc,int8_t HallIo);
int32_t HallSpeedGet(HallCalc_t * pHallCalc);


#endif


