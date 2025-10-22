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
 * @file motordrive.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _MOTORDRIVE_H_
#define _MOTORDRIVE_H_

#include "UserParam.h"
#include "SystemInterface.h"
#include "Svpwm.h"
#include "ErrDeal.h"
#include "DataDeal.h"
#include "CurrentLoop.h"
#include "Brushless.h"
#include "SpeedCtrl.h"
#include "Data_Uart.h"
#include "Hall.h"
#include "EncObserver.h"
#include "SystemDefine.h"
#include "DataDeal.h"
#include "IQmathLib.h"
#include "i2c_drv.h"
#include "oled.h"

typedef struct MOTORBASE
{
	uint16_t CurrentBase;  // Current base value
	uint16_t VolatageBase; // Voltage base value

	// Current base value
	uint16_t OverCurrent;	 // Overcurrent base value
	uint16_t OverVoltage;	 // Overvoltage protection value
	uint16_t LackVoltage;	 // Undervoltage protection value
	uint16_t ElectUpVoltage; // Power supply detection voltage
	// Power-on reset flag
	uint16_t AllowReset; // Allow power-on reset

	uint16_t Fun_DKp_Q13;	  // D-axis Kp in Q13 format
	uint16_t Fun_DKi_Q15;	  // D-axis Ki in Q15 format
	uint16_t Fun_DOutMax_Q15; // D-axis output limit in Q15 format
	uint16_t Fun_QKp_Q13;	  // Q-axis Kp in Q13 format
	uint16_t Fun_QKi_Q15;	  // Q-axis Ki in Q15 format
	uint16_t Fun_QOutMax_Q15; // Q-axis output limit in Q15 format

	// Speed loop parameters
	int16_t Fun_SpdOutMax; // Speed loop output limit
	int16_t Fun_SpdKi;	   // Speed loop Ki
	int16_t Fun_SpdKp;	   // Speed loop Kp

} MotorBase_Obj;
typedef struct
{
	int16_t Motor_pole;
	int16_t Pwm_freq;
} Plate_Obj;
typedef struct _DRIVEROBJ_
{
	Svpwm_Obj Svpwm;
	Plate_Obj Plate;
	MotorFlag_Obj Flag;
	MotorBase_Obj MotorBaseObj;
	Data_Obj DataObj;
	CurLoop_Obj CurLoop;
	Brushless_Obj BrushlessObj;
	PiCtrl_Obj SpeedObj;

	uint16_t SystemCount;
	uint16_t MotorState;
	uint8_t ClearOnce;
	uint16_t SoftStartStopCtrl;
	uint8_t DelayCnt;
	uint8_t VieFlag;
	uint16_t ViewValue[10];

	HallCalc_t HallCalc;
	HallStudy_t HallStudy;

	EncObserver_t EncCalc;
} Motor_Obj;

void FocParamReIni(Motor_Obj *pObj);

#define RUN_STATE 2
#define SHORTPHASE_STATE 1
#define STOP_STATE 0

#endif
