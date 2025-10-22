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
 * @file motordrive.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "MotorDrive.h"
#include "UserParam.h"

Motor_Obj motor_I[2];
RecvFrame mRecvFrame = {0};
/*********************** External Interface Functions *******************************/
void MotorDrive_Init(SystemInterface_Obj *InfObj, SystemInterface_Obj *InfObj1); // Motor drive initialization
void ForMotorMain(SystemInterface_Obj *InfObj, SystemInterface_Obj *InfObj1);
void MotorIsr_200us(uint8_t num, SystemInterface_Obj *InfObj); // Motor 200us interrupt function
void Uart_Isr(uint8_t data_in);
void MotorPwm_Isr_I(uint8_t num, SystemInterface_Obj *InfObj); // Motor PWM interrupt function
void BrakeErrIsr_I(uint8_t num);

/*********************** Internal Interface Functions *******************************/
// Initialization functions
void SvpwmInit(Motor_Obj *Obj, uint16_t *Half_duty_I);
void SvpwmInit_c(Motor_Obj *Obj, uint16_t *Half_duty_I);
void ErrDealInit(Motor_Obj *Obj); // Error handling initialization
void DataDealInit(Motor_Obj *Obj);
void DataDealInit_c(Motor_Obj *Obj);
void CurLoopInit(Motor_Obj *Obj);
void CurLoopInit_c(Motor_Obj *Obj);
void BrushlessInit(Motor_Obj *Obj);
void SpeedCtrlInit(uint8_t num, Motor_Obj *Obj);
void Foc_para_init(Motor_Obj *Obj, Motor_Obj *Obj1);

// Algorithm calculation functions
void SvpwmNew_Isr(uint8_t num, Motor_Obj *Obj, int32_t UdOut_Q15, int32_t UqOut_Q15);
void SvpwmNew_Isr_c(uint8_t num, Motor_Obj *Obj, int32_t UdOut_Q15, int32_t UqOut_Q15);
void StartStop_Isr(Motor_Obj *Obj, uint8_t num); // Start-stop control
void IdIqFbCalcu(Motor_Obj *Obj);				 // Current feedback calculation
void IdIqFbCalcu_c(Motor_Obj *Obj);				 // Current feedback calculation
void CurrentQ15Calcu(Motor_Obj *Obj, SystemInterface_Obj *InfObj);
void CurrentQ15Calcu_c(Motor_Obj *Obj, SystemInterface_Obj *InfObj);
void BrushlessPiCtrl(uint8_t num, Motor_Obj *Obj);
void ElectUpDected(Motor_Obj *Obj);
void ElectUpDected_c(Motor_Obj *Obj);
void Task_Deapare(Motor_Obj *Obj);
void Speed_air_2ms(uint8_t num, Motor_Obj *Obj);
void Speed_air_2ms_IF(uint8_t num, Motor_Obj *Obj);
void Fluxwkn_Run(Motor_Obj *Obj);
void Fluxwkn_Run1(Motor_Obj *Obj);
int16_t SmoForceStartCtrl(Motor_Obj *Obj, uint8_t num);
int8_t GetHallLine_c(HallCalc_t *pHallCalc, int8_t HallIo);
void Observer_Init(Motor_Obj *Obj);
int16_t Observer_Run(uint8_t num, Motor_Obj *Obj);
void SpeedLoop(uint8_t num, Motor_Obj *Obj);

void IFAngle_init(Motor_Obj *Obj); // IF angle initialization

// Timer interrupt handling
void ErrDeal_Isr(Motor_Obj *Obj);			 // Error handling interrupt
void IPhysicalCalcu_100us(Motor_Obj *Obj);	 // Current RMS and peak value calculation
void IPhysicalCalcu_100us_c(Motor_Obj *Obj); // Current RMS and peak value calculation
void IzeroReCalc(Motor_Obj *Obj, SystemInterface_Obj *InfObj);
void IzeroReCalc_c(Motor_Obj *Obj, SystemInterface_Obj *InfObj);
void SpeedGiven(void);

// UART communication
void ExploreParamRoot(Motor_Obj *Obj, Motor_Obj *Obj1, uint8_t *pBuf, int32_t len);
void ExploreParamRoot_c(Motor_Obj *Obj, Motor_Obj *Obj1, uint8_t *pBuf, int32_t len);
void SendFixView(Motor_Obj *Obj, Motor_Obj *Obj1);
void SendFixView_c(Motor_Obj *Obj, Motor_Obj *Obj1);
void OffsetValue_Send(Motor_Obj *Obj);

// SPI communication
extern void DAC_update(uint8_t num, uint16_t data);

// Main loop variables
void Uart_Deal(Motor_Obj *Obj, Motor_Obj *Obj1);
void Spi_Deal(Motor_Obj *Obj, Motor_Obj *Obj1);
void OLED_Deal(Motor_Obj *Obj, Motor_Obj *Obj1);

void SpeedControl(uint8_t num, int32_t TargetSpeed)
{
	motor_I[num].SpeedObj.Speed_Target = TargetSpeed;
}
void StartStopControl(uint8_t num, uint16_t RunEn)
{
	motor_I[num].SoftStartStopCtrl = RunEn;
}
// ========================================================================
// Function Name: MotorDrive_Init
// Description: Motor drive system initialization
// Input Parameters: System interface objects
// Output Parameters: None
// Return Value: None
// Purpose: Motor drive control system initialization
// ========================================================================
void MotorDrive_Init(SystemInterface_Obj *InfObj, SystemInterface_Obj *InfObj1)
{
	/***************motor controller 1 parameters  ******************/
	motor_I[0].Plate.Motor_pole = MOTOR_POPAIRS1;
	motor_I[0].Plate.Pwm_freq = PWM_FREQUENCY1;
	motor_I[0].DelayCnt = 0;
	motor_I[0].SoftStartStopCtrl = 0;
	motor_I[0].SystemCount = 0;
	motor_I[0].ClearOnce = 0;
	motor_I[0].MotorState = STOP_STATE;
	motor_I[0].MotorBaseObj.CurrentBase = CurrentBase1;		  // 1A->0.02V 1.65V, current base value
	motor_I[0].MotorBaseObj.VolatageBase = VolatageBase1;	  // Voltage base value 48V->1.84V
	motor_I[0].MotorBaseObj.OverCurrent = OverCurrent1;		  // Overcurrent threshold, precision 0.1A
	motor_I[0].MotorBaseObj.OverVoltage = OverVoltage1;		  // Overvoltage protection value
	motor_I[0].MotorBaseObj.LackVoltage = LackVoltage1;		  // Undervoltage protection value
	motor_I[0].MotorBaseObj.ElectUpVoltage = ElectUpVoltage1; // Power supply detection voltage 35V

	motor_I[0].MotorBaseObj.AllowReset = AllowReset1;	// Allow power-on reset flag
	motor_I[0].MotorBaseObj.Fun_DKp_Q13 = DKp1;			// D-axis Kp 1500
	motor_I[0].MotorBaseObj.Fun_DKi_Q15 = DKi1;			// D-axis Ki 1000
	motor_I[0].MotorBaseObj.Fun_DOutMax_Q15 = DOutMax1; // D-axis output saturation limit
	motor_I[0].MotorBaseObj.Fun_QKp_Q13 = QKp1;			// Q-axis Kp
	motor_I[0].MotorBaseObj.Fun_QKi_Q15 = QKi1;			// Q-axis Ki
	motor_I[0].MotorBaseObj.Fun_QOutMax_Q15 = QOutMax1; // Q-axis output saturation limit
	motor_I[0].MotorBaseObj.Fun_SpdOutMax = SpdOutMax1; // Speed output saturation limit
	motor_I[0].MotorBaseObj.Fun_SpdKi = SpdKi1;			// Speed loop Ki 400
	motor_I[0].MotorBaseObj.Fun_SpdKp = SpdKp1;			// Speed loop Kp 2000

	motor_I[0].SpeedObj.Speed_Target = 500;
	motor_I[0].SpeedObj.target_calc = 0;
	motor_I[0].SpeedObj.step_target = 20;
	/***************motor controller 2 parameters  ******************/
	motor_I[1].Plate.Motor_pole = MOTOR_POPAIRS2;
	motor_I[1].Plate.Pwm_freq = PWM_FREQUENCY2;
	motor_I[1].DelayCnt = 0;
	motor_I[1].SoftStartStopCtrl = 0;
	motor_I[1].SystemCount = 0;
	motor_I[1].ClearOnce = 0;
	motor_I[1].MotorState = STOP_STATE;
	motor_I[1].MotorBaseObj.CurrentBase = CurrentBase2;		  // 1A->0.02V 1.65V
	motor_I[1].MotorBaseObj.VolatageBase = VolatageBase2;	  // Voltage base value 48V->1.84V
	motor_I[1].MotorBaseObj.OverCurrent = OverCurrent2;		  // Overcurrent threshold, precision 0.1A
	motor_I[1].MotorBaseObj.OverVoltage = OverVoltage2;		  // Overvoltage protection value
	motor_I[1].MotorBaseObj.LackVoltage = LackVoltage2;		  // Undervoltage protection value
	motor_I[1].MotorBaseObj.ElectUpVoltage = ElectUpVoltage2; // Power supply detection voltage 35V

	motor_I[1].MotorBaseObj.AllowReset = AllowReset2;	// 0;
	motor_I[1].MotorBaseObj.Fun_DKp_Q13 = DKp2;			// D-axis Kp 1500
	motor_I[1].MotorBaseObj.Fun_DKi_Q15 = DKi2;			// D-axis Ki 1000
	motor_I[1].MotorBaseObj.Fun_DOutMax_Q15 = QOutMax2; // D-axis output saturation limit
	motor_I[1].MotorBaseObj.Fun_QKp_Q13 = QKp2;			// Q-axis Kp
	motor_I[1].MotorBaseObj.Fun_QKi_Q15 = QKi2;			// Q-axis Ki
	motor_I[1].MotorBaseObj.Fun_QOutMax_Q15 = QOutMax2; // Q-axis output saturation limit
	motor_I[1].MotorBaseObj.Fun_SpdOutMax = SpdOutMax2; // Speed output saturation limit
	motor_I[1].MotorBaseObj.Fun_SpdKi = SpdKi2;			// Speed loop Ki 400
	motor_I[1].MotorBaseObj.Fun_SpdKp = SpdKp2;			// Speed loop Kp 2000

	motor_I[1].SpeedObj.Speed_Target = 500;
	motor_I[1].SpeedObj.target_calc = 0;
	motor_I[1].SpeedObj.step_target = 10;
	/***************motor 1 & motor 2 initial*****************/
	SvpwmInit_c(&motor_I[0], InfObj->Half_duty_I);
	SvpwmInit_c(&motor_I[1], InfObj->Half_duty_I);
	DataDealInit_c(&motor_I[0]);
	DataDealInit_c(&motor_I[1]);
	CurLoopInit_c(&motor_I[0]);
	CurLoopInit_c(&motor_I[1]);
	ErrDealInit(&motor_I[0]);
	ErrDealInit(&motor_I[1]);
	BrushlessInit(&motor_I[0]);
	BrushlessInit(&motor_I[1]);
	SpeedCtrlInit(0, &motor_I[0]);
	SpeedCtrlInit(1, &motor_I[1]);
	Foc_para_init(&motor_I[0], &motor_I[1]);
}
// ========================================================================
// Function Name: Foc_para_init_I
// Description: FOC algorithm parameter initialization
// Input Parameters: Motor object pointers
// Output Parameters: None
// Return Value: None
// Purpose: FOC algorithm parameter initialization
// ========================================================================
void Foc_para_init(Motor_Obj *Obj, Motor_Obj *Obj1)
{
#ifdef HALL_FOR_ANGLE
	/***************	motor 1 parameters  ******************/
	Obj->HallCalc.Pwm_freq = PWM_FREQUENCY1;
	Obj->HallCalc.Pols = MOTOR_POPAIRS1;
	// state parameter
	Obj->HallCalc.HallFixLineSeq[0] = 3;
	Obj->HallCalc.HallFixLineSeq[1] = 1;
	Obj->HallCalc.HallFixLineSeq[2] = 5;
	Obj->HallCalc.HallFixLineSeq[3] = 4;
	Obj->HallCalc.HallFixLineSeq[4] = 6;
	Obj->HallCalc.HallFixLineSeq[5] = 2;
	Obj->HallCalc.HallFixAngle[0] = 24334; // Base angle
	// Call Initiall
	HallIntial(&Obj->HallCalc);
	// Hall Study initial
	Obj->HallStudy.CurrentValue = 300;

	/***************	motor 2 parameters  ******************/
	Obj1->HallCalc.Pwm_freq = PWM_FREQUENCY2;
	Obj1->HallCalc.Pols = MOTOR_POPAIRS2;
	// state parameter
	Obj1->HallCalc.HallFixLineSeq[0] = 3;
	Obj1->HallCalc.HallFixLineSeq[1] = 1;
	Obj1->HallCalc.HallFixLineSeq[2] = 5;
	Obj1->HallCalc.HallFixLineSeq[3] = 4;
	Obj1->HallCalc.HallFixLineSeq[4] = 6;
	Obj1->HallCalc.HallFixLineSeq[5] = 2;
	Obj1->HallCalc.HallFixAngle[0] = 24334; // Base angle
	// Call Initiall
	HallIntial(&Obj1->HallCalc);
	// Hall Study initial
	Obj1->HallStudy.CurrentValue = 300;
#endif
}

void FocLoopReIni(Motor_Obj *pObj)
{
	pObj->SpeedObj.target_calc = 0;
	pObj->SpeedObj.UsDifSum = 0;
	pObj->CurLoop.IdFilterTmp = 0;
	pObj->CurLoop.IqFilterTmp = 0;
	pObj->CurLoop.IdSumTemp = 0;
	pObj->CurLoop.IqSumTemp = 0;
}
void FocParamReIni(Motor_Obj *pObj)
{
	FocLoopReIni(pObj);
}
// ========================================================================
// Function Name: MotorMain_Circle
// Description: Motor main loop processing
// Input Parameters: System interface objects
// Output Parameters: None
// Return Value: None
// Purpose: Motor control main loop function
// ========================================================================
void ForMotorMain(SystemInterface_Obj *InfObj, SystemInterface_Obj *InfObj1)
{
#ifdef UART_SEND
	Uart_Deal(&motor_I[0], &motor_I[1]);
#endif
#ifdef SPI_SEND
	Spi_Deal(&motor_I[0], &motor_I[1]);
#endif
#ifdef Offset_SEND
	OffsetValue_Send(&motor_I[0]);
#endif
}
// ========================================================================
// Function Name: Spi_Deal
// Description: SPI communication data processing
// Input Parameters: Motor object pointers
// Output Parameters: None
// Return Value: None
// Purpose: SPI interface data transmission function
// ========================================================================
extern int16_t TestCount;
void Spi_Deal(Motor_Obj *Obj, Motor_Obj *Obj1)
{
	DAC_update(1, ((Obj->DataObj.Ia_Q15) + 2048));
	DAC_update(2, ((Obj->SpeedObj.Speed_Target >> 4)));
	DAC_update(3, ((Obj->DataObj.Ic_Q15 >> 3) + 2048));
}
// ========================================================================
// Function Name: Uart_Deal
// Description: UART communication data processing
// Input Parameters: Motor object pointers
// Output Parameters: None
// Return Value: None
// Purpose: UART serial communication data transmission function
// ========================================================================
void Uart_Deal(Motor_Obj *Obj, Motor_Obj *Obj1)
{
	SendFixView_c(Obj, Obj1);
	if (mRecvFrame.State == 2) // Data reception complete
	{
		mRecvFrame.index = SubCharFill(mRecvFrame.Buf, mRecvFrame.index);
		if (mRecvFrame.index > 0)
		{
			ExploreParamRoot_c(Obj, Obj1, mRecvFrame.Buf, mRecvFrame.index); //  Data analysis and response processing
		}
		mRecvFrame.State = 0; // Response complete, ready for next frame
	}
	delay_100us(1);
}
// ========================================================================
// Function Name: Uart_Isr
// Description: UART interrupt service routine
// Input Parameters: Received data byte
// Output Parameters: None
// Return Value: None
// Purpose: UART serial communication interrupt function
// ========================================================================
void Uart_Isr(uint8_t data_in)
{
	switch (mRecvFrame.State)
	{
	case 0:
		if (data_in == 0x7E)
		{
			mRecvFrame.State = 1;
			mRecvFrame.index = 0;
		}
		break;
	case 1:
		if (data_in == 0x7E)
		{
			if (mRecvFrame.index > 0)
			{
				mRecvFrame.State = 2;
			}
		}
		else if (mRecvFrame.index < ONE_FRAME_LEN)
		{
			mRecvFrame.Buf[mRecvFrame.index++] = data_in;
		}
		break;
	case 2:
		break;
	default:
		mRecvFrame.State = 0;
	}
}
// ========================================================================
// Function Name: Udc_Deal
// Description: DC bus voltage processing
// Input Parameters: Motor number, motor object, system interface object
// Output Parameters: None
// Return Value: None
// Purpose: DC bus voltage detection and processing function
// ========================================================================
void Udc_Deal(uint8_t num, Motor_Obj *Obj, SystemInterface_Obj *InfObj)
{
	Obj->DataObj.Udc_Q15 = (*InfObj->UdcSample_I << 3);
	Obj->DataObj.Udc_Real = ((uint32_t)Obj->DataObj.Udc_Q15 * Obj->MotorBaseObj.VolatageBase) >> 15;
	if (num == 0)
	{
		Obj->CurLoop.Us2 = (LIMIT_DUTY1 * LIMIT_DUTY1);
	}
	else if (num == 1)
	{
		Obj->CurLoop.Us2 = (LIMIT_DUTY2 * LIMIT_DUTY2);
	}
}
// ========================================================================
// Function Name: MotorIsr_200us
// Description: Motor 200us interrupt service routine
// Input Parameters: Motor number, system interface object
// Output Parameters: None
// Return Value: None
// Purpose: Motor control 200us interrupt function
// ========================================================================
void MotorIsr_200us(uint8_t num, SystemInterface_Obj *InfObj)
{
	motor_I[num].DelayCnt++;
	if (motor_I[num].DelayCnt == 10)
	{
		motor_I[num].DelayCnt = 0;
#ifdef HALL_FOR_ANGLE_T
		Speed_air_2ms(num, &motor_I[num]);
#endif
	}
	else
	{
		ElectUpDected_c(&motor_I[num]);
		StartStop_Isr(&motor_I[num], num);
		ErrDeal_Isr(&motor_I[num]);
		IzeroReCalc_c(&motor_I[num], InfObj);
		SpeedGiven();

#ifdef HALL_FOR_ANGLE_T
		motor_I[num].DataObj.MotorSpeed = HallSpeedGet(&motor_I[num].HallCalc);
#endif
	}
}
// ========================================================================
// Function Name: MotorPwm_Isr_I
// Description: Motor PWM interrupt service routine
// Input Parameters: Motor number, system interface object
// Output Parameters: None
// Return Value: None
// Purpose: Motor control PWM interrupt function
// ========================================================================
void MotorPwm_Isr_I(uint8_t num, SystemInterface_Obj *InfObj)
{
	if (motor_I[num].Flag.Bits.MotorStarted == 1)
	{
		CurrentQ15Calcu_c(&motor_I[num], InfObj);
		IdIqFbCalcu_c(&motor_I[num]);
		BrushlessPiCtrl(num, &motor_I[num]);
#ifdef HALL_FOR_ANGLE_T
		motor_I[num].BrushlessObj.EleAng_Q15 = HallAngleCalc(&motor_I[num].HallCalc, GetIO_GetHall(num));
#endif
		SvpwmNew_Isr_c(num, &motor_I[num], motor_I[num].CurLoop.UdOut_Q15, motor_I[num].CurLoop.UqOut_Q15);
		motor_I[num].ClearOnce = 0;
	}
	else
	{
		if (motor_I[num].ClearOnce < 3)
		{
			motor_I[num].ClearOnce++;
			SvpwmInit_c(&motor_I[num], InfObj->Half_duty_I);
			DataDealInit_c(&motor_I[num]);
			CurLoopInit_c(&motor_I[num]);
			BrushlessInit(&motor_I[num]);
			SpeedCtrlInit(num, &motor_I[num]);

#ifdef HALL_FOR_ANGLE
			HallIntial(&motor_I[num].HallCalc);
#endif
		}
	}
	PwmDutySet_I(num, motor_I[num].Svpwm.DutyA, motor_I[num].Svpwm.DutyB, motor_I[num].Svpwm.DutyC);
}
// ========================================================================
// Function Name: BrakeErrIsr_I
// Description: Brake error interrupt service routine
// Input Parameters: Motor number
// Output Parameters: None
// Return Value: None
// Purpose: Hardware brake error interrupt processing
// ========================================================================
void BrakeErrIsr_I(uint8_t num)
{
	motor_I[num].Flag.Bits.HwTotalErr = 1;
	motor_I[num].Flag.Bits.HwOverCurrent = 1;
}
