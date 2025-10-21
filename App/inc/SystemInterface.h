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
 * @file systeminterface.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _SYSTEMINTERFACE_H_
#define _SYSTEMINTERFACE_H_

#include "stdint.h"
#include "math.h"

#include "n32g430.h"
#include "PublicDefine.h"

typedef struct SYSTEMPARA_OBJ
{
	uint16_t *IaSample_I;   // A���������
	uint16_t *IbSample_I;   // B���������
	uint16_t *IcSample_I;   // C���������
	uint16_t *UdcSample_I;  // ĸ�ߵ�ѹ����
	uint16_t *Half_duty_I;		
}SystemInterface_Obj;
typedef enum
{
	motor1 = 0,
	motor2,
	motor3,
	motor4
}Motor_Num;

extern void PwmDutySet_I(uint8_t num,int16_t duty1,int16_t duty2,int16_t duty3);		// ����ռ�ձ�
extern void SetPwmFreq(uint16_t Freq);                                                  // ����pwmƵ��
extern void AllPwmShut(uint8_t num);                                                    // ����ȫ��
extern void AllPwmOpen(uint8_t num);                                                    // ����ȫ��
extern void MotorIsr_200us(uint8_t num,SystemInterface_Obj *InfObj);                    // �����100us�жϺ���
extern void MotorPwm_Isr_I(uint8_t num,SystemInterface_Obj *InfObj);                    // �����PWM�жϺ���
extern void SpeedControl(uint8_t num,int32_t TargetSpeed);                              //�ٶȿ���
extern void StartStopControl(uint8_t num,uint16_t RunEn);                               //��ͣ״̬����
extern void Uart_Isr(uint8_t data_in);				                                    // �����ж�ִ�к���
extern void MotorDrive_Init(SystemInterface_Obj *InfObj,SystemInterface_Obj *InfObj1);  // ����������ʼ��
extern void BrakeErrIsr_I(uint8_t num);                                                 // ɲ���жϺ���
extern uint16_t GetIO_Button1(void);                                                    // ��ȡ��������״̬
extern void Brake(uint8_t num);					                                        // ɲ��
extern void ForMotorMain(SystemInterface_Obj *InfObj,SystemInterface_Obj *InfObj1);
extern void Send_Value_Uart(uint8_t * pChar,int32_t len);
extern uint8_t GetIO_GetHall(uint8_t num);
extern uint16_t GetIO_GetEnc(uint8_t num);
extern void GetIO_ResetEnc(uint8_t num,uint16_t cnt);                                                // ��λEnc
#endif
