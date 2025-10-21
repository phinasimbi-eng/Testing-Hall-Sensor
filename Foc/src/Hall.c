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
 * @file Hall.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#include <stdint.h>
#include <math.h>
#include "MotorDrive.h"
#include "Hall.h"
#include "string.h"
#include "stdlib.h"

//initiall and reset
void HallIntial(HallCalc_t * pHallCalc)
{
    uint8_t i = 0;
    pHallCalc->AngleCalTickCnt = 0;
    pHallCalc->AngleCalTickInc = 0;
    pHallCalc->HallSpeed = 0;
    pHallCalc->HallSpeedAvg = 0;
    pHallCalc->AngleBaseCntTotal = 0;
    pHallCalc->HallAngle = 0;

    for(;i<MaxSpdBuf;i++)
        pHallCalc->SpeedBuf[i] = 0;
    pHallCalc->Spbi = 0;
    
    pHallCalc->HallAngleOffset = 16384;	//90°
    for(i = 1;i<6;i++)
        pHallCalc->HallFixAngle[i] = pHallCalc->HallFixAngle[i-1]+10922;	//24334  (24334+10922*1)=35256~-30280 46178~-19358(初始位置角) 57100~-8436 68022~2486 78944~13408

    pHallCalc->Delay = 1000;
}


int32_t HallSpeedGet(HallCalc_t * pHallCalc)
{
    int16_t i = 0;
    int32_t sum=0;
    int32_t Speed = 0;
    int32_t AngleCalTickCnt = pHallCalc->AngleCalTickCnt;
    if(AngleCalTickCnt != 0)
    {
        Speed = pHallCalc->Pwm_freq*10/AngleCalTickCnt/pHallCalc->Pols;
        
        if(pHallCalc->Spbi >= MaxSpdBuf)
            pHallCalc->Spbi = 0;
        pHallCalc->SpeedBuf[pHallCalc->Spbi]= Speed;
        pHallCalc->Spbi++;
        for (;i<MaxSpdBuf;i++)
            sum+=pHallCalc->SpeedBuf[i];
        pHallCalc->HallSpeed = sum/MaxSpdBuf;
        pHallCalc->HallSpeedAvg = ((pHallCalc->HallSpeedAvg*63<<4) + (pHallCalc->HallSpeed<<4)) >> 10;	//0.984 0.016 速度滤波
    }
    if(pHallCalc->Delay > 0)
    {
        pHallCalc->Delay--;
        return 0;
    }
    if(pHallCalc->SpeedBuf[MaxSpdBuf-1] != 0)
        return pHallCalc->HallSpeedAvg;  
    return 0;
}


//Hall Angle study
//if you need different speed,you can input different  ForceAngInc,such as 50,100 ...
int16_t HallAngleStudy(HallStudy_t * pHallStudy,int16_t ForceAngInc/*=0*/)
{
    if(ForceAngInc == 0)
    {
        if(pHallStudy->ForceAngInc == 0)
            pHallStudy->ForceAngInc = 1;//default angle inc
    }
    else
    {
        pHallStudy->ForceAngInc = ForceAngInc;
    }
    if(pHallStudy->HallTmpPre != pHallStudy->HallTmp)
    {
        if(pHallStudy->Filter++ > 100)
        {
            pHallStudy->Filter = 0;
            pHallStudy->HallTmpPre = pHallStudy->HallTmp;
            if(pHallStudy->Halli >= HALL_STUDY_LEN)
                pHallStudy->Halli = 0;
            pHallStudy->HallBuf[pHallStudy->Halli] = pHallStudy->HallTmp;
            pHallStudy->AngBuf[pHallStudy->Halli] = pHallStudy->HallAngle;
            pHallStudy->Halli++;
        }
    }
    pHallStudy->HallAngle += pHallStudy->ForceAngInc;//force run to study line sequence and fix angle
    return pHallStudy->HallAngle;
}

//Hall Angle run calc
int16_t HallAngleCalc(HallCalc_t * pHallCalc,int8_t HallIo)
{
    //int16_t HallAngle = 0;
    uint8_t i = 0;
    pHallCalc->HallIo = HallIo;

    for(;i<6;i++)
    {
        if(pHallCalc->HallIo == pHallCalc->HallFixLineSeq[i])
        {
            break;
        }
    }
    if(i >= 6) i = 0;
    pHallCalc->Index = i;
    
    if(pHallCalc->AngleCalTickInc < 1000)//10000/(1000*6) = 1.66Hz 在霍尔信号发生变化后清零
        pHallCalc->AngleCalTickInc++;
    else
        pHallCalc->PosNegCnt = 0;
    
    if(pHallCalc->IndexOld != pHallCalc->Index)
    {
        uint8_t tmp = pHallCalc->Index - pHallCalc->IndexOld;
        pHallCalc->AngleCalTickCntPre = pHallCalc->AngleCalTickInc;
        int32_t cnt = pHallCalc->AngleCalTickCnt;
        pHallCalc->AngleCalTickCnt = (tmp == 1 || tmp == 251)?pHallCalc->AngleCalTickInc:-pHallCalc->AngleCalTickInc;	//正反转判定并赋值测速Cnt	
				if(pHallCalc->AngleCalTickCnt >= -5 && pHallCalc->AngleCalTickCnt <= 5)
				{
					pHallCalc->AngleCalTickCnt = 1000;
				}			
        if((cnt > 0 && pHallCalc->AngleCalTickCnt > 0) || (cnt < 0 && pHallCalc->AngleCalTickCnt < 0)) //上一次计数和本次计数都存在 霍尔信号发生了变化
        {
            if(pHallCalc->PosNegCnt < 100)
                pHallCalc->PosNegCnt++;
        }
        else pHallCalc->PosNegCnt = 0;
        pHallCalc->AngleCalTickInc = 0;
        pHallCalc->IndexOld = pHallCalc->Index;
        pHallCalc->AngleBaseCntTotal++;
    }
    
    
    if(pHallCalc->PosNegCnt >= 2)
    {
        int32_t IncSet = 0;
        if(pHallCalc->AngleCalTickInc < pHallCalc->AngleCalTickCntPre)	//霍尔信号发生了变化
        {
            IncSet = 10922*pHallCalc->AngleCalTickInc / pHallCalc->AngleCalTickCntPre;
        }
        else 
            IncSet = 10922;	//60°
        
        if(pHallCalc->AngleCalTickCnt > 0)//Angel Inc
            pHallCalc->HallAngle = pHallCalc->HallFixAngle[pHallCalc->Index] + pHallCalc->HallAngleOffset  + IncSet;	//x + 90° + 60°或0°
        else//Angel Dec
            pHallCalc->HallAngle = pHallCalc->HallFixAngle[pHallCalc->IndexOld] + pHallCalc->HallAngleOffset  - IncSet;
    }    
    else
    {
        pHallCalc->HallAngle = pHallCalc->HallFixAngle[pHallCalc->Index] + pHallCalc->HallAngleOffset + 5461;	//-19357初始位置角
    }
    
    //pHallCalc->HallAngle = HallAngle;
    return pHallCalc->HallAngle;
}





