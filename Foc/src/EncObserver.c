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
 * @file EncObserver.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
 
#include <stdlib.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "EncObserver.h"


void EncIntial(struct EncObserver_t * pObserver)
{    
    memset(&pObserver->Angle,0, (int32_t)&pObserver->end - (int32_t)&pObserver->Angle);
    pObserver->MultC = 65536*pObserver->Pols*16/pObserver->PulsPerPeriod;
}
int16_t EncSpeedGet(struct EncObserver_t *pObserver)
{    
    //检查转了的Puls
    if(pObserver->CalAngleCntBak != 0)
    {
        int32_t tmp;
        tmp = 16l*pObserver->Pols*pObserver->Pwm*pObserver->RuningPulsCntBak/pObserver->PulsPerPeriod/pObserver->CalAngleCntBak;
        pObserver->CalSpeedHz = tmp;//realtim angel spead hz            
        
        //恢复
        pObserver->CalAngleCntBak = 0;
        
        //计算RPM
        tmp = tmp*60/pObserver->Pols;
        pObserver->CalSpeedRpm = (pObserver->CalSpeedRpm*7>>3) + ((tmp<<12)>>3);
    }
    return pObserver->CalSpeedRpm>>16;
}
int16_t EncAngleCalc(struct EncObserver_t *pObserver,uint16_t GetEnc)
{
    pObserver->PreEncodePuls = pObserver->EncodePuls;
    pObserver->EncodePuls = GetEnc;
    
    //Pulse计数
    int32_t temp = pObserver->EncodePuls - pObserver->PreEncodePuls;
    if(abs(temp) > (pObserver->PulsPerPeriod>>1))
    {
        if(pObserver->EncodePuls > pObserver->PreEncodePuls)
            temp = (-pObserver->PulsPerPeriod + pObserver->EncodePuls) - pObserver->PreEncodePuls;
        else
            temp = (pObserver->PulsPerPeriod + pObserver->EncodePuls) - pObserver->PreEncodePuls;
    }
    pObserver->RuningPulsCnt = pObserver->RuningPulsCnt + temp;
    pObserver->RuningPulsExCnt = pObserver->RuningPulsExCnt + temp; 
    //执行次数
    pObserver->CalAngleCnt++;
    pObserver->CalAngleExCnt++;
    
    if(pObserver->CalAngleCntBak == 0)
    {
        pObserver->RuningPulsCntBak = pObserver->RuningPulsCnt;
        pObserver->CalAngleCntBak = pObserver->CalAngleCnt;
        pObserver->RuningPulsCnt = 0;
        pObserver->CalAngleCnt = 0;
    }
    
    pObserver->Angle = (pObserver->EncodePuls*pObserver->MultC)>>4; 
    return pObserver->Angle;
}

//用于位置环的物理圈数信息
void EndGetPosition(struct EncObserver_t *pObserver,int32_t *CalAngleExCnt,int32_t *RuningPulsExCnt)
{
    *CalAngleExCnt = pObserver->CalAngleExCnt;
    *RuningPulsExCnt = pObserver->RuningPulsExCnt;
}

//extern call
//中断函数调用
void EncIrqSetting(struct EncObserver_t *pObserver,uint16_t Cnt)
{ 
    ;
}
