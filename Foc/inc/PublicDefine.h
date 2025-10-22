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
 * @file publicdefine.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _PUBLICDEFINE_H_
#define _PUBLICDEFINE_H_
#include "stdint.h"
#include "math.h"
#include "UserParam.h"

/* Other Define */
#define MTR_MAXRPM (3000)						  // Mechanical maximum speed
#define MTR_MINRPM (0)							  // Mechanical minimum speed
#define MTR_VOLT_RPM (float)(MTR_MAXRPM / 4096.0) // Voltage to speed conversion coefficient

/* hardware current sample Parameter */
#define HW_RSHUNT (0.01) // 0.01 (Ω)  Current sense resistance
#define HW_ADC_REF (3.3) // 3.3  (V)  ADC reference voltage
#define HW_AMPGAIN (4.0) // 4.0 Operational amplifier gain
#define I_ValueX(Curr_Value) ((Curr_Value) * (HW_RSHUNT) * (HW_AMPGAIN) / (HW_ADC_REF))
#define I_Value(Curr_Value) _IQ(I_ValueX(Curr_Value))
#define HW_BOARD_CURR_MAX (HW_ADC_REF / 2 / HW_AMPGAIN / HW_RSHUNT) // Maximum detectable current 41.25A
#define HW_BOARD_CURR_MIN (-HW_BOARD_CURR_MAX)						// Minimum detectable current -41.25A
#define HW_BOARD_CURR_BASE (HW_BOARD_CURR_MAX * 2)					// Current base value 82.5A

/* hardware voltage sample Parameter */
#define RV1 (15.0)										  // 15 (kΩ) Bus voltage divider resistor 1
#define RV2 (0.0)										  // 0.0 (kΩ) Bus voltage divider resistor 2
#define RV3 (1.0)										  // 1.0 (kΩ) Bus voltage divider resistor 3
#define RV ((RV1 + RV2 + RV3) / RV3)					  // Voltage divider ratio
#define HW_BOARD_VOLT_MAX (HW_ADC_REF * RV)				  // (V)  Maximum ADC measurable bus voltage 52.8V
#define HW_BOARD_VOLTAGE_BASE (HW_BOARD_VOLT_MAX / 1.732) // Voltage base value 30.48V

/*********************PCBA Configuration*********************/
#define MAIN_FREQUENCY (128000000l) // Main frequency
// motor 1
#define PWM_FREQUENCY1 10000l // PWM frequency
#define CURRENT_BASE1 (41.2)  // Current base value
#define VOLTAGE_BASE1 (25.2)  // Voltage base value
#define LIMIT_DUTY1 32000l	  // Duty cycle limit, max 32767l

/*************** Motor Parameters*********************/
#define FI1 (0.016566)		 // Flux linkage VPP/(2*2*pi*sqrt(3)*f)
#define MOTOR_POPAIRS1 (2.0) // Motor pole pairs
#define LD1 (0.0037 / 2)	 // D-axis inductance, L per unit value at 1KHz divided by 2
#define LQ1 (0.0037 / 2)	 // Q-axis inductance
#define RS1 (4.7 / 2)		 // Stator resistance, R per unit value at 100Hz divided by 2

/*************** Control Parameters*********************/
// motor 1
#define CurrentBase1 (CURRENT_BASE1 * 10) // Current base value in 0.1A units for calculating actual current, not used
#define VolatageBase1 (53)				  // Voltage base value, ADC count per 1V 3.3V*(1+15) 1K 15K voltage divider

#define OverCurrent1 I_Value(10.0) // Overcurrent protection value 10.0 A
#define OverVoltage1 (32)		   // Overvoltage protection value V
#define LackVoltage1 (12)		   // Undervoltage protection value V
#define ElectUpVoltage1 (12)	   // Power supply detection voltage V
#define AllowReset1 (0)			   // Allow power-on reset flag

#define DKp1 _IQ12(0.4)			// D-axis Kp in Q12 format
#define DKi1 _IQ(0.02)			// D-axis Ki in Q15 format
#define QKp1 _IQ12(0.4)			// Q-axis Kp in Q12 format
#define QKi1 _IQ(0.02)			// Q-axis Ki in Q15 format
#define DOutMax1 _IQ(0.99)		// D-axis output limit in Q15 format
#define QOutMax1 _IQ(0.99)		// Q-axis output limit in Q15 format
#define SpdKp1 _IQ12(0.2)		// Speed loop Kp in Q15 format
#define SpdKi1 _IQ12(0.01)		// Speed loop Ki in Q15 format
#define SpdOutMax1 I_Value(4.0) // Speed loop output limit

/*************** Communication Settings*********************/
/* USART send and receive setting */
#define UART_SEND // UART transmission
// #define SPI_SEND  			//SPI transmission
// #define Offset_SEND 		//Need to transmit via UART or SPI

#define SEND_INT
#define BUFFNUM (100) // Buffer size for data transmission count per cycle
#ifdef SEND_FLOAT
#define BUFFSIZE ((BUFFNUM * 4 * 3) + 2) // Communication buffer size ((BUFFNUM*4*m)+2) m is data transmission multiplier "+2" represents frame header '0X7E' and frame tail '0X7E'
#endif
#ifdef SEND_INT
#define BUFFSIZE ((BUFFNUM * 2 * 6) + 2) // Communication buffer size ((BUFFNUM*2*n)+2) n is data transmission multiplier "+2" represents frame header '0X7E' and frame tail '0X7E'
#endif
#define BUFFSIZE1 (38) // Parameter data buffer size 4*9+2

/* filter parameters */
#define FilterNum (200) // Average filter count
#define CalcuNum (200)	// Calculation count

/* Other parameters */
#define SIN_RAD 0x0300
#define U0_90 0x0000
#define U90_180 0x0100
#define U180_270 0x0200
#define U270_360 0x0300

#define IQSin_Cos_DEFAULTS {0, 0, 0}
#define IQAtan_DEFAULTS {0, 0, 0, 0, 0}

typedef struct
{
	int32_t IQAngle; // Input:   alpha-axis
	int32_t IQSin;	 // Input:   beta-axis
	int32_t IQCos;	 // Output:  phase-a
} IQSin_Cos, *p_IQSin_Cos;

typedef struct
{
	int32_t Alpha; // Input:   alpha-axis
	int32_t Beta;  // Input:   beta-axis
	int32_t IQTan; // Output:  phase-a
	int32_t IQAngle;
	int32_t JZIQAngle;
} IQAtan, *p_IQAtan;

extern int16_t Sinlt(int16_t Angle);						   // Lookup table sine function
extern int16_t Coslt(int16_t Angle);						   // Lookup table cosine function
extern int32_t LLimit(int32_t Data, int32_t Min, int32_t Max); // 32-bit limiting function
extern int16_t Limit(int16_t Data, int16_t Min, int16_t Max);  // 16-bit limiting function
extern int16_t Labs(int16_t Data);							   // 16-bit absolute value function
extern int32_t LLabs(int32_t Data);							   // 32-bit absolute value function
extern uint16_t sqrt_16(uint32_t M);
extern int16_t LowPassFilter(int16_t Data, int16_t Factor, int32_t *FilterObserve); // Low-pass filter function

#endif
