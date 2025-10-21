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
#define _Q7(A)                          (int8) ((A) * (128))
#define _Q10(A)                         (int16)((A) * (1024))                                                         
#define _Q11(A)                         (int16)((A) * (2048))                                                     
#define _Q12(A)                         (int16)((A) * (4096))                                                   
#define _Q13(A)                         (int16)((A) * (8192))                                                         
#define _Q15(A)                         (int16)((A) * (32767))                                                          
#define _2PI                            (3.1415926 * 2)                                                               
#define _Q16                            (65535.0)                                                                     

/* -----Private define----- */
#define I_ValueX(Curr_Value)            ((Curr_Value) * (HW_RSHUNT) * (HW_AMPGAIN) / (HW_ADC_REF))
#define I_Value(Curr_Value)             _Q15(I_ValueX(Curr_Value))
#define QOUTVALUE                    	  I_Value(QOUTCURRENT)                                                            


/* -----硬件板子参数设置值----- */
#define RV                              ((RV1 + RV2 + RV3) / RV3)                                                        // 分压比
#define HW_BOARD_VOLT_MAX               (HW_ADC_REF * RV)                                                                // (V)  ADC可测得的最大母线电压

/* -----正常运行时估算算法的参数设置值----- */
#define OBS_KSLIDE                      _Q15(0.85)                                                                       // SMO算法里的滑膜增益值
#define E_BW_Wind                       (600.0)//(BASE_FREQ*2)                                                           // PLL算法里的反电动势滤波值
#define E_BW                            (400.0)//(BASE_FREQ*2)                                                           // PLL算法里的反电动势滤波值

/* -----芯片参数值----- */
/* -----CPU and PWM Parameter----- */
#define PWM_CYCLE                       (1000.0 / PWM_FREQUENCY)                                                         // 周期us
#define SAMP_FREQ                       (PWM_FREQUENCY * 1000)                                                           // 采样频率(HZ)
#define TPWM_VALUE                      (1.0 / SAMP_FREQ)                                                                // 载波周期(S)

/* -----初始位置检查参数----- */
#define PosCheckEnable                  (0)                                                                              // 初始位置使能

/* -----脉冲注入时间长于2ms 或 低于2ms----- */
#define InjectTime                      (Short_Inject)
#define RPD_Time                        (3)                                                                              // (ms) 每次RPD的时间
#define RPD_CurValue                    (6.5)                                                                            // (A)  RPD过流值
#define DAC_RPDCurValue                 _Q7(I_ValueX(RPD_CurValue * 2))

#define ATT_COEF                        (0.85)                                                                           // 无需改动


#define   UDQMAX_Volt_VALUE    			    _Q15(Under_Protect_Voltage/HW_BOARD_VOLT_MAX)
#define   UDQMIN_Volt_VALUE    			    _Q15(Over_Protect_Voltage/HW_BOARD_VOLT_MAX)
#define   UDQ_K		                      ((float)(UDQMAX-UDQMIN)/(float)(UDQMAX_Volt_VALUE-UDQMIN_Volt_VALUE))            //2498

/* -----过温保护值设置----- */
#define   Tempera_Value(NTC_Value) 		  _Q15((5.0*NTC_Value/(4.7+NTC_Value))/4.5)									                       // 10K上拉电阻时，NTC阻值对应Q15_AD值，单位：KΩ

/* -----double resistor sample Parameter----- */
#define DLL_TIME                        (1.0)                                                                            // 双电阻最小脉宽设置(us),建议值为死区时间值+0.2us以上

/* -----three resistor overmodule Parameter----- */
#define OVERMOD_TIME                    (2.0)                                                                            // 三电阻过调制时间(us)，建议值2.0

/* -----deadtime compensation----- */
#define DT_TIME                         (0.0)                                                                            // 死区补偿时间(us)，适用于双电阻和三电阻，建议值是1/2死区时间

/* -----*min pulse----- */
#define GLI_TIME                        (0.0)                                                                            // 桥臂窄脉宽消除(us),建议值0.5

/* -----deadtime Parameter----- */
#define PWM_LOAD_DEADTIME               (PWM_DEADTIME * MCU_CLOCK)                                                       // 死区设置值
#define PWM_OVERMODULE_TIME             (OVERMOD_TIME * MCU_CLOCK / 2)                                                   // 过调制时间
#define PWM_DLOWL_TIME                  (DLL_TIME * MCU_CLOCK / 2)                                                       //下桥臂最小时间

/* -----single resistor sample Parameter----- */
#define PWM_TS_LOAD                     (uint16)(_Q16 / PWM_CYCLE * MIN_WIND_TIME / 16)                                  // 单电阻采样设置值
#define PWM_DT_LOAD                     (uint16)(_Q16 / PWM_CYCLE * DT_TIME / 16)                                        // 死区补偿值
//#define PWM_TGLI_LOAD                 (uint16)(_Q16 / PWM_CYCLE * (GLI_TIME + PWM_DEADTIME) / 16)                      // 最小脉冲

#define PWM_TGLI_LOAD                   (uint16)(_Q16 / PWM_CYCLE * (GLI_TIME) / 16)                                     // 最小脉冲

/* -----硬件板子参数设置值----- */
/* -----hardware current sample Parameter----- */
/* -----电流基准的电路参数----- */
#define HW_BOARD_CURR_MAX               (HW_ADC_REF / 2 / HW_AMPGAIN / HW_RSHUNT)                                        // 最大采样电流,2.702A
#define HW_BOARD_CURR_MIN               (-HW_BOARD_CURR_MAX)                                                             // 最小采样电流,-2.702A
#define HW_BOARD_CURR_BASE              (HW_BOARD_CURR_MAX * 2)                                                          // 电流基准//5.4A

/* -----hardware voltage sample Parameter----- */
/* -----母线电压采样分压电路参数----- */
#define HW_BOARD_VOLTAGE_BASE           (HW_BOARD_VOLT_MAX / 1.732)                                                      // 电压基准
#define HW_BOARD_VOLTAGE_VC             ((RV1 + RV2 + RV3 * VC1) / (RV3 * VC1))
#define HW_BOARD_VOLTAGE_BASE_Start     (HW_ADC_REF * HW_BOARD_VOLTAGE_VC / 1.732)                                       // 电压基准

/* -----硬件过流保护DAC值----- */
#define DAC_OvercurrentValue            (_Q7(I_ValueX((OverHardcurrentValue)*(2))) +(0x7F))                              //此处*2因为是Q7格式

#define Align_Theta                     _Q15((float)Align_Angle / 180.0)
#define BEMF_Theta                      _Q15((float)0.0 / 180.0)

#define BASE_FREQ                       ((MOTOR_SPEED_BASE / 60) * Pole_Pairs)                                           // 基准频率

/* -----保护参数值----- */
/* -----protect value----- */
#define OVER_PROTECT_VALUE              _Q15(Over_Protect_Voltage  / HW_BOARD_VOLT_MAX)
#define UNDER_PROTECT_VALUE             _Q15(Under_Protect_Voltage / HW_BOARD_VOLT_MAX)
#define OVER_RECOVER_VALUE              _Q15(Over_Recover_Vlotage  / HW_BOARD_VOLT_MAX)
#define UNDER_RECOVER_VALUE             _Q15(Under_Recover_Vlotage / HW_BOARD_VOLT_MAX)

/* -----motor speed set value----- */
#define Motor_Open_Ramp_ACC             _Q15(MOTOR_OPEN_ACC     / MOTOR_SPEED_BASE)
#define Motor_Open_Ramp_Min             _Q15(MOTOR_OPEN_ACC_MIN / MOTOR_SPEED_BASE)

#define Motor_Omega_Ramp_Min            _Q15(MOTOR_OMEGA_ACC_MIN / MOTOR_SPEED_BASE)
#define Motor_Omega_Ramp_End            _Q15(MOTOR_OMEGA_ACC_END / MOTOR_SPEED_BASE)

#define Motor_Loop_Speed                _Q15(MOTOR_LOOP_RPM / MOTOR_SPEED_BASE)

#define Motor_Max_Speed                 _Q15(MOTOR_SPEED_MAX_RPM   / MOTOR_SPEED_BASE)
#define Motor_Min_Speed                 _Q15(MOTOR_SPEED_MIN_RPM   / MOTOR_SPEED_BASE)
#define Motor_Limit_Speed               _Q15(MOTOR_SPEED_LIMIT_RPM / MOTOR_SPEED_BASE)
#define Motor_Stop_Speed                _Q15(MOTOR_SPEED_STOP_RPM  / MOTOR_SPEED_BASE)

#define Motor_Stall_Min_Speed           _Q15(MOTOR_SPEED_STAL_MIN_RPM / MOTOR_SPEED_BASE)
#define Motor_Stall_Max_Speed           _Q15(MOTOR_SPEED_STAL_MAX_RPM / MOTOR_SPEED_BASE)

#define Motor_RD_Speed                  _Q15(MOTOR_SPEED_RD_RPM    / MOTOR_SPEED_BASE)
#define Motor_RDPT_Speed                _Q15(MOTOR_SPEED_RDPT_RPM  / MOTOR_SPEED_BASE)
#define Motor_RDRCV_Speed               _Q15(MOTOR_SPEED_RDRCV_RPM / MOTOR_SPEED_BASE)


#define Motor_Over_Speed1                _Q15(MOTOR_SPEED_OVER_RPM1   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed2                _Q15(MOTOR_SPEED_OVER_RPM2   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed3                _Q15(MOTOR_SPEED_OVER_RPM3   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed4                _Q15(MOTOR_SPEED_OVER_RPM4   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed5                _Q15(MOTOR_SPEED_OVER_RPM5   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed6                _Q15(MOTOR_SPEED_OVER_RPM6   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed7                _Q15(MOTOR_SPEED_OVER_RPM7   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed8                _Q15(MOTOR_SPEED_OVER_RPM8   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed9                _Q15(MOTOR_SPEED_OVER_RPM9   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed10               _Q15(MOTOR_SPEED_OVER_RPM10   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed11               _Q15(MOTOR_SPEED_OVER_RPM11   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed12               _Q15(MOTOR_SPEED_OVER_RPM12   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed13               _Q15(MOTOR_SPEED_OVER_RPM13   / MOTOR_SPEED_BASE)
#define Motor_Over_Speed14               _Q15(MOTOR_SPEED_OVER_RPM14   / MOTOR_SPEED_BASE)
#define Motor_Over_RecoverSpeed         _Q15(MOTOR_SPEED_OVER_RecoverRPM   / MOTOR_SPEED_BASE)

#define SPEED_K                         ((float)(Motor_Max_Speed-Motor_Min_Speed)/(float)(MAXPWMDuty-MINPWMDuty))
#define POWER_K                         ((float)(Motor_Max_Power-Motor_Min_Power)/(float)(MAXPWMDuty-MINPWMDuty))

/* -----obsever parameter set value----- */
#define MAX_BEMF_VOLTAGE                ((MOTOR_SPEED_BASE*Ke)/(1000.0))
#define MAX_OMEG_RAD_SEC                ((float)(_2PI*BASE_FREQ))
#define OBS_K1T                         _Q15(LD/(LD+RS*TPWM_VALUE))
#define OBS_K2T                         _Q13((TPWM_VALUE/(LD+RS*TPWM_VALUE))*(HW_BOARD_VOLTAGE_BASE_Start/HW_BOARD_CURR_BASE))
#define OBS_K2T_SMO                     _Q13((TPWM_VALUE/(LD+RS*TPWM_VALUE))*1.4*(HW_BOARD_VOLTAGE_BASE_Start/HW_BOARD_CURR_BASE))
#define OBS_K2T_Actual                  _Q13((TPWM_VALUE/(LD+RS*TPWM_VALUE))*(HW_BOARD_VOLTAGE_BASE/HW_BOARD_CURR_BASE))
#define OBS_K3T                         _Q15((TPWM_VALUE/(LD+RS*TPWM_VALUE))*(MAX_BEMF_VOLTAGE/HW_BOARD_CURR_BASE))
#define OBS_K4T                         _Q15(((LD-LQ)*TPWM_VALUE*MAX_OMEG_RAD_SEC)/(LD+RS*TPWM_VALUE))

#define OBSW_KP_GAIN                    _Q12(2*_2PI*ATT_COEF*ATO_BW/BASE_FREQ) //0.08
#define OBSW_KI_GAIN                    _Q15(_2PI*ATO_BW*ATO_BW*TPWM_VALUE/BASE_FREQ) //0.00003

#define OBSW_KP_GAIN_RUN                _Q12(2*_2PI*ATT_COEF*ATO_BW_RUN/BASE_FREQ)
#define OBSW_KI_GAIN_RUN                _Q15(_2PI*ATO_BW_RUN*ATO_BW_RUN*TPWM_VALUE/BASE_FREQ)

#define OBSW_KP_GAIN_RUN1               _Q12(2*_2PI*ATT_COEF*ATO_BW_RUN1/BASE_FREQ)
#define OBSW_KI_GAIN_RUN1               _Q15(_2PI*ATO_BW_RUN1*ATO_BW_RUN1*TPWM_VALUE/BASE_FREQ)

#define OBSW_KP_GAIN_RUN2               _Q12(2*_2PI*ATT_COEF*ATO_BW_RUN2/BASE_FREQ)
#define OBSW_KI_GAIN_RUN2               _Q15(_2PI*ATO_BW_RUN2*ATO_BW_RUN2*TPWM_VALUE/BASE_FREQ)

#define OBSW_KP_GAIN_RUN3               _Q12(2*_2PI*ATT_COEF*ATO_BW_RUN3/BASE_FREQ)
#define OBSW_KI_GAIN_RUN3               _Q15(_2PI*ATO_BW_RUN3*ATO_BW_RUN3*TPWM_VALUE/BASE_FREQ)

#define OBSW_KP_GAIN_RUN4               _Q12(2*_2PI*ATT_COEF*ATO_BW_RUN4/BASE_FREQ)
#define OBSW_KI_GAIN_RUN4               _Q15(_2PI*ATO_BW_RUN4*ATO_BW_RUN4*TPWM_VALUE/BASE_FREQ)

#define OBS_FBASE                       BASE_FREQ*TPWM_VALUE*32768                                                       // Fbase*Tpwm*32768
#define OBS_KLPF                        _Q15(_2PI*BASE_FREQ*TPWM_VALUE)                                                  // 2PI*Fbase*Tpwm
#define SPEED_KLPF                      _Q15(_2PI*SPD_BW*TPWM_VALUE)                                                     // 2PI*SPD_BW*Tpwm
#define OBS_EA_KS                       _Q15((2*MOTOR_SPEED_SMOMIN_RPM*_2PI*BASE_FREQ*TPWM_VALUE)/MOTOR_SPEED_BASE)      // SMO的最小速度

#define OBSE_PLLKP_GAIN_WIND            _Q11(((2*ATT_COEF*_2PI*E_BW_Wind*LD - RS)*HW_BOARD_CURR_BASE)/HW_BOARD_VOLTAGE_BASE)
#define OBSE_PLLKI_GAIN_WIND            _Q11((_2PI*E_BW_Wind*_2PI*E_BW_Wind*LD*TPWM_VALUE*HW_BOARD_CURR_BASE)/HW_BOARD_VOLTAGE_BASE)

#define OBSE_PLLKP_GAIN                 _Q11(((2*ATT_COEF*_2PI*E_BW*LD - RS)*HW_BOARD_CURR_BASE)/HW_BOARD_VOLTAGE_BASE) //0.16
#define OBSE_PLLKI_GAIN                 _Q11((_2PI*E_BW*_2PI*E_BW*LD*TPWM_VALUE*HW_BOARD_CURR_BASE)/HW_BOARD_VOLTAGE_BASE) //0.0087

/* -----逆风判断时的估算算法设置值----- */
#define SPEED_KLPF_WIND                 _Q15(_2PI*SPD_BW_Wind*TPWM_VALUE)                                                // 2PI*SPD_BW_Wind*Tpwm
#define OBSW_KP_GAIN_WIND               _Q12(2*_2PI*ATT_COEF*ATO_BW_Wind/BASE_FREQ)
//#define   OBSW_KI_GAIN_WIND           _Q15(_2PI*0.5*ATO_BW_Wind*ATO_BW_Wind*TPWM_VALUE/BASE_FREQ)                      //---SMO
#define OBSW_KI_GAIN_WIND               _Q15(_2PI*ATO_BW_Wind*ATO_BW_Wind*TPWM_VALUE/BASE_FREQ)                          //---PLL

/* -----Current Calib:enable or disable----- */
#define CalibDisable                    (0)                                                                              //
#define CalibEnable                     (1)                                                                              //
#define CalibENDIS                      (CalibEnable)

/* -----SVPWM mode----- */
#define SVPWM_5_Segment                 (0)                                                                              // 五段式SVPWM
#define SVPWM_7_Segment                 (1)                                                                              // 七段式SVPWM
#define SVPMW_Mode                      (SVPWM_7_Segment)

/* -----double resistor sample mode----- */
#define DouRes_1_Cycle                  (0)                                                                              // 1 周期采样完 ia, ib
#define DouRes_2_Cycle                  (1)                                                                              // 交替采用ia, ib, 2周期采样完成
#define DouRes_Sample_Mode              (DouRes_1_Cycle)

/* -----PWM high or low level Mode----- */
/* -----根据驱动芯片的类型选择，大部分芯片为High_Level----- */
#define High_Level                      (0)                                                                              // 驱动高电平有效
#define Low_Level                       (1)                                                                              // 驱动低电平有效
#define UP_H_DOWN_L                     (2)                                                                              // 上桥臂高电平有效，下桥臂低电平有效
#define UP_L_DOWN_H                     (3)                                                                              // 上桥臂低电平有效，下桥臂高电平有效

/* -----脉冲注入时间长于2ms 或 低于2ms----- */
#define Long_Inject                     (0)                                                                              // 脉冲注入时间长于2ms,若时间长于4ms，则要修改定时器分频
#define Short_Inject                    (1)                                                                              // 脉冲注入时间低于2ms

/* -----调速模式----- */
#define PWMMODE                         (0)                                                                              // PWM调速
#define SREFMODE                        (1)                                                                              // 模拟调速
#define NONEMODE                        (2)                                                                              // 直接给定值，不调速
#define KEYMODE                         (3)                                                                              // 按键调速模式

/* -----IPM测试模式----- */
#define IPMtest                         (0)                                                                              // IPM测试或者MOS测试，MCU输出固定占空比
#define NormalRun                       (1)                                                                              // 正常按电机状态机运行

/* -----估算器模式选择----- */
#define SMO                             (0)                                                                              // SMO ,滑膜估算
#define PLL                             (1)                                                                              // PLL ,锁相环

/* -----顺逆风判断设置----- */
#define NoTailWind                      (0)                                                                              // 无逆风顺风判断
#define TailWind                        (1)                                                                              // 逆风顺风判断

/* -----顺逆风判断方法----- */
#define RSDMethod                       (0)                                                                              // RSD比较器方法
#define BEMFMethod                      (1)                                                                              // BEMF方法


/* -----开环启动模式选择----- */
#define Open_Start                      (0)                                                                              // 开环强拖启动
#define Omega_Start                     (1)                                                                              // Omega启动
#define Open_Omega_Start                (2)                                                                              // 先开环启，后Omega启动

/* -----电流采样模式----- */
#define Single_Resistor                 (0)                                                                              // 单电阻电流采样模式
#define Double_Resistor                 (1)                                                                              // 双电阻电流采样模式
#define Three_Resistor                  (2)                                                                              // 三电阻电流采样模式

/* -----硬件过流保护----- */
#define Hardware_FO_Protect             (1)                                                                              // 硬件FO过流保护使能，适用于IPM有FO保护的场合
#define Hardware_CMP_Protect            (2)                                                                              // 硬件CMP比较过流保护使能，适用于MOS管应用场合
#define Hardware_FO_CMP_Protect         (3)                                                                              // 硬件CMP比较和FO过流保护都使能
#define Hardware_Protect_Disable        (4)                                                                              // 硬件过流保护禁止，用于测试

/* -----硬件过流保护比较值来源----- */

#define Compare_DAC                     (0)                                                                              // DAC设置硬件过流值
#define Compare_Hardware                (1)                                                                              // 硬件设置硬件过流值

/* -----外环选择功率环或速度环----- */
#define POWER_LOOP_CONTROL              (0)                                                                              //恒功率
#define SPEED_LOOP_CONTROL              (1)                                                                              //恒转速

/* -----外环使能----- */
#define OUTLoop_Disable                 (0)                                                                              // 关闭外环
#define OUTLoop_Enable                  (1)                                                                              // 使能外环

/* -----正PWMduty or 负PWMduty Choose----- */
#define PosiPWMDUTY			                (0)														                                                   // 正PWMduty
#define NegaPWMDUTY				              (1)														                                                   // 负PWMduty

#endif