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
 * @file errdeal.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _ERRDEAL_H_
#define _ERRDEAL_H_

#include "stdint.h"
// ������϶���---------------------------------------------------------------
typedef union _MOTOR_FLAG_ALL_
{
    uint16_t Arr[2];
    struct
    {
			uint16_t SwOverCurrent :1;                // �������
			uint16_t HwTotalErr    :1;                // Ӳ������
			uint16_t HwOverCurrent :1;                // Ӳ������
			uint16_t LowTempFlag	 :1;             		// ���±�����־
			uint16_t HighTempFlag  :1;								// ���±�����־
			uint16_t LackVolatage  :1;                // Ƿѹ����
			uint16_t OverVolatage  :1;                // ��ѹ����
			uint16_t IZeroFlag     :1;                // ��Ư����
			uint16_t Strcv0	    	 :8;
			
			uint16_t MotorStarted  :1;                // �������
			uint16_t ShortPhase    :1;                // �����·��־
			uint16_t AllowStart    :1;                // ��������
			uint16_t MotorStartReq :1;                // ��������
			uint16_t ElectUpFlag   :1;                // �ϵ绺���ѹ��־
			uint16_t DutyErr		   :1;                // ռ�ձȼ������
			uint16_t SystemReady   :1;                // ϵͳ�ȶ���־
			uint16_t SlidingFlag   :1;                // ���м���־
			uint16_t OverModulateFlag :1;             // ����
			uint16_t ForceStartFlag  :1;							// ǿ��������־
			
			uint16_t BrakeFlag  :1;							// ǿ��������־
			uint16_t UpperControl  :1;							// ��λ����ͣ����
			uint16_t FOCControl  :1;							// ѹ������ͣ����
			uint16_t Strcv    			: 3;
    } Bits;
} MotorFlag_Obj;



#endif 
