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

/* -----Define to prevent recursive inclusion----- */
#ifndef __DEVELOP_H_
#define __DEVELOP_H_

/* -----Define to prevent recursive inclusion----- */
/* -----Q format define----- */
#define _Q7(A) (int8)((A) * (128))
#define _Q10(A) (int16)((A) * (1024))
#define _Q11(A) (int16)((A) * (2048))
#define _Q12(A) (int16)((A) * (4096))
#define _Q13(A) (int16)((A) * (8192))
#define _Q15(A) (int16)((A) * (32767))
#define _2PI (3.1415926 * 2)
#define _Q16 (65535.0)

/* -----Private define----- */
#define I_ValueX(Curr_Value) ((Curr_Value) * (HW_RSHUNT) * (HW_AMPGAIN) / (HW_ADC_REF))
#define I_Value(Curr_Value) _Q15(I_ValueX(Curr_Value))
#define QOUTVALUE I_Value(QOUTCURRENT)

/* -----Hardware Board Parameter Settings----- */
#define RV ((RV1 + RV2 + RV3) / RV3)        // Voltage divider ratio
#define HW_BOARD_VOLT_MAX (HW_ADC_REF * RV) // (V) Maximum bus voltage measurable by ADC

/* -----Normal Operation Estimation Algorithm Parameter Settings----- */
#define OBS_KSLIDE _Q15(0.85) // Sliding mode gain value in SMO algorithm
#define E_BW_Wind (600.0)     //(BASE_FREQ*2)                                                           // Back EMF filter value in PLL algorithm
#define E_BW (400.0)          //(BASE_FREQ*2)                                                           // Back EMF filter value in PLL algorithm

/* -----Chip Parameter Values----- */
/* -----CPU and PWM Parameter----- */
#define PWM_CYCLE (1000.0 / PWM_FREQUENCY) // Period in microseconds
#define SAMP_FREQ (PWM_FREQUENCY * 1000)   // Sampling frequency (Hz)
#define TPWM_VALUE (1.0 / SAMP_FREQ)       // Carrier period (S)

/* -----Initial Position Check Parameters----- */
#define PosCheckEnable (0) // Initial position enable

/* -----Pulse Injection Time Greater Than 2ms or Less Than 2ms----- */
#define InjectTime (Short_Inject)
#define RPD_Time (3)       // (ms) Time duration for each RPD
#define RPD_CurValue (6.5) // (A) RPD overcurrent value
#define DAC_RPDCurValue _Q7(I_ValueX(RPD_CurValue * 2))

#define ATT_COEF (0.85) // No modification needed

#define UDQMAX_Volt_VALUE _Q15(Under_Protect_Voltage / HW_BOARD_VOLT_MAX)
#define UDQMIN_Volt_VALUE _Q15(Over_Protect_Voltage / HW_BOARD_VOLT_MAX)
#define UDQ_K ((float)(UDQMAX - UDQMIN) / (float)(UDQMAX_Volt_VALUE - UDQMIN_Volt_VALUE)) // 2498

/* -----Over-temperature Protection Settings----- */
#define Tempera_Value(NTC_Value) _Q15((5.0 * NTC_Value / (4.7 + NTC_Value)) / 4.5) // NTC resistance to Q15_AD value with 10K pull-up resistor, unit: KΩ

/* -----double resistor sample Parameter----- */
#define DLL_TIME (1.0) // Dual resistor minimum pulse width setting (us), recommended value is dead time + 0.2us or more

/* -----three resistor overmodule Parameter----- */
#define OVERMOD_TIME (2.0) // Three resistor over-modulation time (us), recommended value 2.0

/* -----deadtime compensation----- */
#define DT_TIME (0.0) // Dead time compensation (us), applicable to dual and three resistor, recommended value is 1/2 dead time

/* -----*min pulse----- */
#define GLI_TIME (0.0) // Bridge arm narrow pulse width elimination (us), recommended value 0.5

/* -----deadtime Parameter----- */
#define PWM_LOAD_DEADTIME (PWM_DEADTIME * MCU_CLOCK)       // Dead time setting value
#define PWM_OVERMODULE_TIME (OVERMOD_TIME * MCU_CLOCK / 2) // Over-modulation time
#define PWM_DLOWL_TIME (DLL_TIME * MCU_CLOCK / 2)          // Lower bridge arm minimum time

/* -----single resistor sample Parameter----- */
#define PWM_TS_LOAD (uint16)(_Q16 / PWM_CYCLE * MIN_WIND_TIME / 16) // Single resistor sampling setting value
#define PWM_DT_LOAD (uint16)(_Q16 / PWM_CYCLE * DT_TIME / 16)       // Dead time compensation value
// #define PWM_TGLI_LOAD                 (uint16)(_Q16 / PWM_CYCLE * (GLI_TIME + PWM_DEADTIME) / 16)                      // Minimum pulse

#define PWM_TGLI_LOAD (uint16)(_Q16 / PWM_CYCLE * (GLI_TIME) / 16) // Minimum pulse

/* -----Hardware Board Parameter Settings----- */
/* -----hardware current sample Parameter----- */
/* -----Current Reference Circuit Parameters----- */
#define HW_BOARD_CURR_MAX (HW_ADC_REF / 2 / HW_AMPGAIN / HW_RSHUNT) // Maximum sampling current, 2.702A
#define HW_BOARD_CURR_MIN (-HW_BOARD_CURR_MAX)                      // Minimum sampling current, -2.702A
#define HW_BOARD_CURR_BASE (HW_BOARD_CURR_MAX * 2)                  // Current reference // 5.4A

/* -----hardware voltage sample Parameter----- */
/* -----Bus Voltage Sampling Divider Circuit Parameters----- */
#define HW_BOARD_VOLTAGE_BASE (HW_BOARD_VOLT_MAX / 1.732) // Voltage reference
#define HW_BOARD_VOLTAGE_VC ((RV1 + RV2 + RV3 * VC1) / (RV3 * VC1))
#define HW_BOARD_VOLTAGE_BASE_Start (HW_ADC_REF * HW_BOARD_VOLTAGE_VC / 1.732) // Voltage reference

/* -----Hardware Over-current Protection DAC Value----- */
#define DAC_OvercurrentValue (_Q7(I_ValueX((OverHardcurrentValue) * (2))) + (0x7F)) // *2 here because it's Q7 format

#define Align_Theta _Q15((float)Align_Angle / 180.0)
#define BEMF_Theta _Q15((float)0.0 / 180.0)

#define BASE_FREQ ((MOTOR_SPEED_BASE / 60) * Pole_Pairs) // Base frequency

/* -----Protection Parameter Values----- */
/* -----protect value----- */
#define OVER_PROTECT_VALUE _Q15(Over_Protect_Voltage / HW_BOARD_VOLT_MAX)
#define UNDER_PROTECT_VALUE _Q15(Under_Protect_Voltage / HW_BOARD_VOLT_MAX)
#define OVER_RECOVER_VALUE _Q15(Over_Recover_Vlotage / HW_BOARD_VOLT_MAX)
#define UNDER_RECOVER_VALUE _Q15(Under_Recover_Vlotage / HW_BOARD_VOLT_MAX)

/* -----motor speed set value----- */
#define Motor_Open_Ramp_ACC _Q15(MOTOR_OPEN_ACC / MOTOR_SPEED_BASE)
#define Motor_Open_Ramp_Min _Q15(MOTOR_OPEN_ACC_MIN / MOTOR_SPEED_BASE)

#define Motor_Omega_Ramp_Min _Q15(MOTOR_OMEGA_ACC_MIN / MOTOR_SPEED_BASE)
#define Motor_Omega_Ramp_End _Q15(MOTOR_OMEGA_ACC_END / MOTOR_SPEED_BASE)

#define Motor_Loop_Speed _Q15(MOTOR_LOOP_RPM / MOTOR_SPEED_BASE)

#define Motor_Max_Speed _Q15(MOTOR_SPEED_MAX_RPM / MOTOR_SPEED_BASE)
#define Motor_Min_Speed _Q15(MOTOR_SPEED_MIN_RPM / MOTOR_SPEED_BASE)
#define Motor_Limit_Speed _Q15(MOTOR_SPEED_LIMIT_RPM / MOTOR_SPEED_BASE)
#define Motor_Stop_Speed _Q15(MOTOR_SPEED_STOP_RPM / MOTOR_SPEED_BASE)

#define Motor_Stall_Min_Speed _Q15(MOTOR_SPEED_STAL_MIN_RPM / MOTOR_SPEED_BASE)
#define Motor_Stall_Max_Speed _Q15(MOTOR_SPEED_STAL_MAX_RPM / MOTOR_SPEED_BASE)

#define Motor_RD_Speed _Q15(MOTOR_SPEED_RD_RPM / MOTOR_SPEED_BASE)
#define Motor_RDPT_Speed _Q15(MOTOR_SPEED_RDPT_RPM / MOTOR_SPEED_BASE)
#define Motor_RDRCV_Speed _Q15(MOTOR_SPEED_RDRCV_RPM / MOTOR_SPEED_BASE)

#define Motor_Over_Speed1 _Q15(MOTOR_SPEED_OVER_RPM1 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed2 _Q15(MOTOR_SPEED_OVER_RPM2 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed3 _Q15(MOTOR_SPEED_OVER_RPM3 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed4 _Q15(MOTOR_SPEED_OVER_RPM4 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed5 _Q15(MOTOR_SPEED_OVER_RPM5 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed6 _Q15(MOTOR_SPEED_OVER_RPM6 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed7 _Q15(MOTOR_SPEED_OVER_RPM7 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed8 _Q15(MOTOR_SPEED_OVER_RPM8 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed9 _Q15(MOTOR_SPEED_OVER_RPM9 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed10 _Q15(MOTOR_SPEED_OVER_RPM10 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed11 _Q15(MOTOR_SPEED_OVER_RPM11 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed12 _Q15(MOTOR_SPEED_OVER_RPM12 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed13 _Q15(MOTOR_SPEED_OVER_RPM13 / MOTOR_SPEED_BASE)
#define Motor_Over_Speed14 _Q15(MOTOR_SPEED_OVER_RPM14 / MOTOR_SPEED_BASE)
#define Motor_Over_RecoverSpeed _Q15(MOTOR_SPEED_OVER_RecoverRPM / MOTOR_SPEED_BASE)

#define SPEED_K ((float)(Motor_Max_Speed - Motor_Min_Speed) / (float)(MAXPWMDuty - MINPWMDuty))
#define POWER_K ((float)(Motor_Max_Power - Motor_Min_Power) / (float)(MAXPWMDuty - MINPWMDuty))

/* -----obsever parameter set value----- */
#define MAX_BEMF_VOLTAGE ((MOTOR_SPEED_BASE * Ke) / (1000.0))
#define MAX_OMEG_RAD_SEC ((float)(_2PI * BASE_FREQ))
#define OBS_K1T _Q15(LD / (LD + RS * TPWM_VALUE))
#define OBS_K2T _Q13((TPWM_VALUE / (LD + RS * TPWM_VALUE)) * (HW_BOARD_VOLTAGE_BASE_Start / HW_BOARD_CURR_BASE))
#define OBS_K2T_SMO _Q13((TPWM_VALUE / (LD + RS * TPWM_VALUE)) * 1.4 * (HW_BOARD_VOLTAGE_BASE_Start / HW_BOARD_CURR_BASE))
#define OBS_K2T_Actual _Q13((TPWM_VALUE / (LD + RS * TPWM_VALUE)) * (HW_BOARD_VOLTAGE_BASE / HW_BOARD_CURR_BASE))
#define OBS_K3T _Q15((TPWM_VALUE / (LD + RS * TPWM_VALUE)) * (MAX_BEMF_VOLTAGE / HW_BOARD_CURR_BASE))
#define OBS_K4T _Q15(((LD - LQ) * TPWM_VALUE * MAX_OMEG_RAD_SEC) / (LD + RS * TPWM_VALUE))

#define OBSW_KP_GAIN _Q12(2 * _2PI * ATT_COEF * ATO_BW / BASE_FREQ)        // 0.08
#define OBSW_KI_GAIN _Q15(_2PI * ATO_BW * ATO_BW * TPWM_VALUE / BASE_FREQ) // 0.00003

#define OBSW_KP_GAIN_RUN _Q12(2 * _2PI * ATT_COEF * ATO_BW_RUN / BASE_FREQ)
#define OBSW_KI_GAIN_RUN _Q15(_2PI * ATO_BW_RUN * ATO_BW_RUN * TPWM_VALUE / BASE_FREQ)

#define OBSW_KP_GAIN_RUN1 _Q12(2 * _2PI * ATT_COEF * ATO_BW_RUN1 / BASE_FREQ)
#define OBSW_KI_GAIN_RUN1 _Q15(_2PI * ATO_BW_RUN1 * ATO_BW_RUN1 * TPWM_VALUE / BASE_FREQ)

#define OBSW_KP_GAIN_RUN2 _Q12(2 * _2PI * ATT_COEF * ATO_BW_RUN2 / BASE_FREQ)
#define OBSW_KI_GAIN_RUN2 _Q15(_2PI * ATO_BW_RUN2 * ATO_BW_RUN2 * TPWM_VALUE / BASE_FREQ)

#define OBSW_KP_GAIN_RUN3 _Q12(2 * _2PI * ATT_COEF * ATO_BW_RUN3 / BASE_FREQ)
#define OBSW_KI_GAIN_RUN3 _Q15(_2PI * ATO_BW_RUN3 * ATO_BW_RUN3 * TPWM_VALUE / BASE_FREQ)

#define OBSW_KP_GAIN_RUN4 _Q12(2 * _2PI * ATT_COEF * ATO_BW_RUN4 / BASE_FREQ)
#define OBSW_KI_GAIN_RUN4 _Q15(_2PI * ATO_BW_RUN4 * ATO_BW_RUN4 * TPWM_VALUE / BASE_FREQ)

#define OBS_FBASE BASE_FREQ * TPWM_VALUE * 32768                                                        // Fbase*Tpwm*32768
#define OBS_KLPF _Q15(_2PI * BASE_FREQ * TPWM_VALUE)                                                    // 2PI*Fbase*Tpwm
#define SPEED_KLPF _Q15(_2PI * SPD_BW * TPWM_VALUE)                                                     // 2PI*SPD_BW*Tpwm
#define OBS_EA_KS _Q15((2 * MOTOR_SPEED_SMOMIN_RPM * _2PI * BASE_FREQ * TPWM_VALUE) / MOTOR_SPEED_BASE) // SMO minimum speed

#define OBSE_PLLKP_GAIN_WIND _Q11(((2 * ATT_COEF * _2PI * E_BW_Wind * LD - RS) * HW_BOARD_CURR_BASE) / HW_BOARD_VOLTAGE_BASE)
#define OBSE_PLLKI_GAIN_WIND _Q11((_2PI * E_BW_Wind * _2PI * E_BW_Wind * LD * TPWM_VALUE * HW_BOARD_CURR_BASE) / HW_BOARD_VOLTAGE_BASE)

#define OBSE_PLLKP_GAIN _Q11(((2 * ATT_COEF * _2PI * E_BW * LD - RS) * HW_BOARD_CURR_BASE) / HW_BOARD_VOLTAGE_BASE)      // 0.16
#define OBSE_PLLKI_GAIN _Q11((_2PI * E_BW * _2PI * E_BW * LD * TPWM_VALUE * HW_BOARD_CURR_BASE) / HW_BOARD_VOLTAGE_BASE) // 0.0087

/* -----Estimation Algorithm Settings for Reverse Wind Detection----- */
#define SPEED_KLPF_WIND _Q15(_2PI * SPD_BW_Wind * TPWM_VALUE) // 2PI*SPD_BW_Wind*Tpwm
#define OBSW_KP_GAIN_WIND _Q12(2 * _2PI * ATT_COEF * ATO_BW_Wind / BASE_FREQ)
// #define   OBSW_KI_GAIN_WIND           _Q15(_2PI*0.5*ATO_BW_Wind*ATO_BW_Wind*TPWM_VALUE/BASE_FREQ)                      //---SMO
#define OBSW_KI_GAIN_WIND _Q15(_2PI * ATO_BW_Wind * ATO_BW_Wind * TPWM_VALUE / BASE_FREQ) //---PLL

/* -----Current Calib:enable or disable----- */
#define CalibDisable (0) //
#define CalibEnable (1)  //
#define CalibENDIS (CalibEnable)

/* -----SVPWM mode----- */
#define SVPWM_5_Segment (0) // 5-segment SVPWM
#define SVPWM_7_Segment (1) // 7-segment SVPWM
#define SVPMW_Mode (SVPWM_7_Segment)

/* -----double resistor sample mode----- */
#define DouRes_1_Cycle (0) // Complete sampling of ia, ib in 1 cycle
#define DouRes_2_Cycle (1) // Alternating sampling of ia, ib, completed in 2 cycles
#define DouRes_Sample_Mode (DouRes_1_Cycle)

/* -----PWM high or low level Mode----- */
/* -----Select based on driver chip type, most chips use High_Level----- */
#define High_Level (0)  // Drive high level active
#define Low_Level (1)   // Drive low level active
#define UP_H_DOWN_L (2) // Upper bridge arm high level active, lower bridge arm low level active
#define UP_L_DOWN_H (3) // Upper bridge arm low level active, lower bridge arm high level active

/* -----Pulse Injection Time Greater Than 2ms or Less Than 2ms----- */
#define Long_Inject (0)  // Pulse injection time greater than 2ms, if time exceeds 4ms, timer prescaler needs modification
#define Short_Inject (1) // Pulse injection time less than 2ms

/* -----Speed Control Mode----- */
#define PWMMODE (0)  // PWM speed control
#define SREFMODE (1) // Analog speed control
#define NONEMODE (2) // Direct setpoint mode, no speed control
#define KEYMODE (3)  // Key-based speed control mode

/* -----IPM Test Mode----- */
#define IPMtest (0)   // IPM test or MOS test, MCU outputs fixed duty cycle
#define NormalRun (1) // Normal motor state machine operation

/* -----Estimator Mode Selection----- */
#define SMO (0) // SMO, sliding mode observer
#define PLL (1) // PLL, phase-locked loop

/* -----Wind Direction Detection Settings----- */
#define NoTailWind (0) // No tailwind/headwind detection
#define TailWind (1)   // Tailwind/headwind detection enabled

/* -----Wind Direction Detection Method----- */
#define RSDMethod (0)  // RSD comparator method
#define BEMFMethod (1) // Back-EMF method

/* -----Open Loop Startup Mode Selection----- */
#define Open_Start (0)       // Open loop forced startup
#define Omega_Start (1)      // Omega startup mode
#define Open_Omega_Start (2) // Open loop first, then Omega startup

/* -----Current Sampling Mode----- */
#define Single_Resistor (0) // Single resistor current sampling mode
#define Double_Resistor (1) // Dual resistor current sampling mode
#define Three_Resistor (2)  // Three resistor current sampling mode

/* -----Hardware Overcurrent Protection----- */
#define Hardware_FO_Protect (1)      // Hardware FO overcurrent protection enable, suitable for IPM with FO protection
#define Hardware_CMP_Protect (2)     // Hardware CMP comparator overcurrent protection enable, suitable for MOSFET applications
#define Hardware_FO_CMP_Protect (3)  // Both hardware CMP and FO overcurrent protection enabled
#define Hardware_Protect_Disable (4) // Hardware overcurrent protection disabled, for testing purposes

/* -----Hardware Overcurrent Protection Comparison Value Source----- */

#define Compare_DAC (0)      // DAC sets hardware overcurrent value
#define Compare_Hardware (1) // Hardware sets hardware overcurrent value

/* -----Outer Loop Selection: Power Loop or Speed Loop----- */
#define POWER_LOOP_CONTROL (0) // Constant power
#define SPEED_LOOP_CONTROL (1) // Constant speed

/* -----Outer Loop Enable----- */
#define OUTLoop_Disable (0) // Disable outer loop
#define OUTLoop_Enable (1)  // Enable outer loop

/* -----Positive PWM Duty or Negative PWM Duty Choose----- */
#define PosiPWMDUTY (0) // Positive PWM duty
#define NegaPWMDUTY (1) // Negative PWM duty

#endif