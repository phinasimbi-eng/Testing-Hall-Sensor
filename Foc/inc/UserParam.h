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
 * @file userparam.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _USERPARAM_H_
#define _USERPARAM_H_

#define HALL_FOR_ANGLE   // HALL sensing (1)		(Only effective when both (1) and (2) are enabled)
#define HALL_FOR_ANGLE_T // HALL sensing (2)		(Only effective when both (1) and (2) are enabled)

/*********************Control Mode Selection*********************/
/* Note: Torque control mode */

// #define TORQUE_CONTROL        // Torque control mode
#define SPEED_CONTROL // Speed control mode

///*********************Demo Mode Selection *********************/
#define PCBA_SPEED_DEMO // Speed control unit demo

/*********************PCBA Configuration*********************/
// motor 2
#define PWM_FREQUENCY2 10000l // PWM frequency
#define CURRENT_BASE2 (16.5f) // Current base value
#define VOLTAGE_BASE2 (24.0f) // Voltage base value
#define LIMIT_DUTY2 30000l    // Duty cycle limit, max 32767l

/*********************Motor Parameters*********************/
// motor 2
#define MOTOR_POPAIRS2 (2) // Motor pole pairs
#define LD2 (0.000925f)    // D-axis inductance
#define LQ2 (0.000925f)    // Q-axis inductance
#define RS2 (0.916f / 2)   // Stator resistance
#define FI2 (0.01374f)     // Flux linkage
#define ABZ_PULS2 (4096)   // ABZ encoder pulse count per revolution

// motor 2
#define CurrentBase2 (CURRENT_BASE1 * 10) // Current base value in 0.1A units
#define VolatageBase2 (53)                // Voltage base value, ADC count per 1V

#define OverCurrent2 (140)   // Overcurrent protection value
#define OverVoltage2 (45)    // Overvoltage protection value
#define LackVoltage2 (12)    // Undervoltage protection value
#define ElectUpVoltage2 (12) // Power supply detection voltage
#define AllowReset2 (0)      // Allow power-on reset flag

#define DKp2 (2500)        // D-axis Kp in Q13 format
#define DKi2 (1000)        // D-axis Ki in Q15 format
#define DOutMax2 (15000)   // D-axis output limit in Q15 format
#define QKp2 (2500)        // Q-axis Kp in Q13 format
#define QKi2 (1000)        // Q-axis Ki in Q15 format
#define QOutMax2 (15000)   // Q-axis output limit in Q15 format
#define SpdKi2 (1000)      // Speed loop Ki //(5000)
#define SpdKp2 (2000)      // Speed loop Kp //(15000)
#define SpdOutMax2 (10000) // Speed loop output limit

/*****************Torque Control Parameters *******************************/
// motor 1
#define MAX_DEF_TORQUE1 3000 // Maximum default torque control current in Q15 format
#define STEP_TORQUE1 50
// motor 2
#define MAX_DEF_TORQUE2 3000 // Maximum default torque control current in Q15 format
#define STEP_TORQUE2 50

/*****************end *******************************/
#endif
