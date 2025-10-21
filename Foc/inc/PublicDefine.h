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
#define     MTR_MAXRPM								 	 (3000)															 //电位器最大速度
#define     MTR_MINRPM								 	 (0)															   //电位器最小速度
#define     MTR_VOLT_RPM								 (float)(MTR_MAXRPM/4096.0)					 //电压转化速度系数

/* hardware current sample Parameter */
#define 		HW_RSHUNT                    (0.01)                   	    //0.01 (Ω)  采样电阻
#define 		HW_ADC_REF                   (3.3)                          //3.3  (V)  ADC参考电压
#define 		HW_AMPGAIN                   (4.0)                          //4.0 运放放大倍数
#define 		I_ValueX(Curr_Value)         ((Curr_Value) * (HW_RSHUNT) * (HW_AMPGAIN) / (HW_ADC_REF))
#define 		I_Value(Curr_Value)          _IQ(I_ValueX(Curr_Value))
#define 		HW_BOARD_CURR_MAX            (HW_ADC_REF / 2 / HW_AMPGAIN / HW_RSHUNT)   // 最大采样电流 41.25A
#define 		HW_BOARD_CURR_MIN            (-HW_BOARD_CURR_MAX)                        // 最小采样电流 -41.25A
#define 		HW_BOARD_CURR_BASE           (HW_BOARD_CURR_MAX * 2)                     // 电流基准 82.5A

/* hardware voltage sample Parameter */
#define 		RV1                          (15.0)                          //15 (kΩ) 母线电压分压电阻1
#define 		RV2                          (0.0)                           //0.0 (kΩ) 母线电压分压电阻2
#define 		RV3                          (1.0)                           //1.0 (kΩ) 母线电压分压电阻3
#define 		RV                           ((RV1 + RV2 + RV3) / RV3)       // 分压比
#define 		HW_BOARD_VOLT_MAX            (HW_ADC_REF * RV)               // (V)  ADC可测得的最大母线电压 52.8V
#define 		HW_BOARD_VOLTAGE_BASE        (HW_BOARD_VOLT_MAX / 1.732)     // 电压基准 30.48V

/*********************PCBA参数*********************/
#define MAIN_FREQUENCY	    (128000000l) 	    // 主频
//motor 1
#define PWM_FREQUENCY1		10000l		    // 电机载频
#define CURRENT_BASE1 		(41.2)				// 电流基值
#define VOLTAGE_BASE1 	  (25.2)			  // 电压基值
#define LIMIT_DUTY1				32000l			  // 占空比限制 max 32767l 

/*************** 电机参数*********************/
#define FI1 				    (0.016566)  	// 磁链 VPP/(2*2*pi*sqrt(3)*f)
#define MOTOR_POPAIRS1 	(2.0)				  // 电机极对数
#define LD1 					  (0.0037/2)    // D轴相电感 电桥1KHz时L挡位测量值除以2
#define LQ1 					  (0.0037/2)    // Q轴相电感 
#define RS1 					  (4.7/2) 	    // 定子相电阻 电桥100Hz时R挡位测量值除以2

/*************** 调制参数*********************/
//motor 1
#define CurrentBase1   		(CURRENT_BASE1*10)      // 电流基值精度0.1A，用于计算实际电流 not used
#define VolatageBase1  		(53)  	                // 电压基值 精度1V，ADC能采到的最大值 3.3V*(1+15) 1K 15K分压电阻

#define OverCurrent1    		I_Value(10.0)  // 过流保护值 10.0 A
#define OverVoltage1    		(32) 						// 过压保护值 V
#define LackVoltage1    		(12) 						// 欠压保护值 V
#define ElectUpVoltage1 		(12) 						// 上电缓冲电压 V
#define AllowReset1   			(0)							// 允许故障复位标志


#define DKp1  					_IQ12(0.4)	  // D轴KpQ12
#define DKi1  					_IQ(0.02)	  // D轴KiQ15
#define QKp1  					_IQ12(0.4)	  // Q轴KpQ12
#define QKi1  					_IQ(0.02)	  // Q轴KiQ15
#define DOutMax1 				_IQ(0.99)	    // D轴限幅Q15
#define QOutMax1 				_IQ(0.99)	    // Q轴限幅Q15
#define SpdKp1			    _IQ12(0.2)	  // 转速环KpQ15
#define SpdKi1			    _IQ12(0.01)	  // 转速环KiQ15
#define SpdOutMax1   		I_Value(4.0)	// 转速环输出限幅

/*************** 其它参数*********************/
/* USART send and receive setting */
#define UART_SEND   	//串口发送
//#define SPI_SEND  			//SPI发送
//#define Offset_SEND 		//需要定义串口或SPI

#define 		SEND_INT 
#define 	  BUFFNUM                      (100)							//单个变量每次发送数据个数
#ifdef 		  SEND_FLOAT
#define     BUFFSIZE                     ((BUFFNUM*4*3)+2)	//发送数据总量设置 ((BUFFNUM*4*m)+2) m为数据发送变量个数 "+2"表示数据帧头'0X7E'和帧尾'0X7E'”
#endif
#ifdef      SEND_INT				
#define     BUFFSIZE                     ((BUFFNUM*2*6)+2)	//发送数据总量设置 ((BUFFNUM*2*n)+2) n为数据发送变量个数 "+2"表示数据帧头'0X7E'和帧尾'0X7E'”
#endif
#define     BUFFSIZE1                    (38)               //接收数据缓冲个数 4*9+2 

/* filter parameters */
#define   	FilterNum                    (200)	 //平均值滤波数组
#define   	CalcuNum                     (200)   //计算次数

/* Other parameters */
#define SIN_RAD     0x0300
#define U0_90       0x0000
#define U90_180     0x0100
#define U180_270    0x0200
#define U270_360    0x0300

#define IQSin_Cos_DEFAULTS  { 0,0,0}
#define IQAtan_DEFAULTS  {0,0,0,0,0}



typedef struct 	{ 
	        int32_t  IQAngle; 			// Input:   alpha-axis
				  int32_t  IQSin;			// Input:   beta-axis
				  int32_t  IQCos;				// Output:  phase-a		 
				} IQSin_Cos , *p_IQSin_Cos;



typedef struct 	{ 
	        int32_t  Alpha; 			// Input:   alpha-axis
				  int32_t  Beta;			// Input:   beta-axis
				  int32_t  IQTan;				// Output:  phase-a	
  				int32_t  IQAngle;		
	        int32_t  JZIQAngle;
         } IQAtan , *p_IQAtan;


extern int16_t Sinlt(int16_t Angle); // 正弦查表函数
extern int16_t Coslt(int16_t Angle); // 余弦查表函数
extern int32_t LLimit(int32_t Data,int32_t Min,int32_t Max); // 32位限幅函数
extern int16_t Limit(int16_t Data,int16_t Min,int16_t Max); // 16位限幅函数
extern int16_t Labs(int16_t Data);   									  // 16位绝对值函数
extern int32_t LLabs(int32_t Data);   									// 32位绝对值函数
extern uint16_t sqrt_16(uint32_t M);
extern int16_t LowPassFilter(int16_t Data,int16_t Factor,int32_t *FilterObserve);    // 低通滤波函数

#endif
