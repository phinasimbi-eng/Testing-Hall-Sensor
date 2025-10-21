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
    int16_t ZeroCal_ReadyCnt;  // ��Ư�ȶ�����
    int32_t ZeroCal_ASum;      // A����Ư��ֵ�ۻ�
    int32_t ZeroCal_BSum;      // B����Ư��ֵ�ۻ�
    int32_t ZeroCal_CSum;      // C����Ư��ֵ�ۻ�

    int16_t ZeroCal_Cnt;       // ��Ư�������
    int32_t ZeroCal_ArrASum;   // �����˲������
    int32_t ZeroCal_ArrBSum;   // �����˲������
    int32_t ZeroCal_ArrCSum;   // �����˲������

    int16_t ZeroCal_ArrIndex;  // �����˲�����
    int16_t ZeroCal_ArrA[16];  // �����˲�����A
    int16_t ZeroCal_ArrB[16];  // �����˲�����B
    int16_t ZeroCal_ArrC[16];  // �����˲�����C

    int16_t ZeroRecal_Cnt;     // ��Ư�������
    int16_t Ia_ZeroRef;        // A����Ư�ο�ֵ
    int16_t Ib_ZeroRef;        // B����Ư�ο�ֵ
    int16_t Ic_ZeroRef;        // C����Ư�ο�ֵ

    int16_t IMaxCalcuCnt;      // �������ֵ������
    int16_t Ia_MaxBuff;        // A�����ֵ�ݴ���
    int16_t Ib_MaxBuff;        // B�����ֵ�ݴ���
    int16_t Ic_MaxBuff;        // C�����ֵ�ݴ���

    int16_t Udc_Q15;           // ĸ�ߵ�ѹQ15
    int16_t Udc_Real;          // ĸ�ߵ�ѹʵ��ֵ
    int32_t Ia_Q15;            // A�����Q15
    int32_t Ib_Q15;            // B�����Q15
    int32_t Ic_Q15;            // C�����Q15
//    int16_t Ia_Phy;            // A���������ֵ
//    int16_t Ib_Phy;            // B���������ֵ
//    int16_t Ic_Phy;            // C���������ֵ
    float Ia_Phy;            // A���������ֵ
    float Ib_Phy;            // B���������ֵ
    float Ic_Phy;            // C���������ֵ

    int16_t Ia_PhyMax;         // A��������ֵ
    int16_t Ib_PhyMax;         // B��������ֵ
    int16_t Ic_PhyMax;         // C��������ֵ

    int32_t MotorSpeed;        // ���ת��

    int16_t IdObj;             // IdĿ��ֵ
    int16_t IqObj;             // IqĿ��ֵ
				
	int16_t  PhaseAVolZeroRef; // A���ѹQ12
	int16_t  PhaseBVolZeroRef; // B���ѹQ12
	int16_t  PhaseCVolZeroRef; // C���ѹQ12
				
                
		
	uint16_t CurrentCnt;
	uint16_t OverVoltageCnt;
	uint16_t LackVoltageCnt;
	
	uint16_t HwTotalErrCnt;	
} Data_Obj;


#endif 
