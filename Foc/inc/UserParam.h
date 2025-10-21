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

#define HALL_FOR_ANGLE            //HALL有感(1)		(只能同时定义(1)和(2)才生效)
#define HALL_FOR_ANGLE_T          //HALL有感(2)		(只能同时定义(1)和(2)才生效)

/*********************选择工作模式*********************/
/* 注:暂无转矩模式 */

//#define TORQUE_CONTROL        //转矩模式
#define SPEED_CONTROL           //速度模式

///*********************交互控制选择 *********************/
#define PCBA_SPEED_DEMO         //板载电位器调速

/*********************PCBA参数*********************/
//motor 2
#define PWM_FREQUENCY2		10000l		    // 电机载频
#define CURRENT_BASE2 		(16.5f)			// 电流基值
#define VOLTAGE_BASE2 	    (24.0f)			// 电压基值
#define LIMIT_DUTY2			30000l			// 占空比限限制 max 32767l 

/*********************电机参数*********************/
//motor 2
#define MOTOR_POPAIRS2 			(2)			    // 电机极对数
#define LD2 				    (0.000925f)     // D轴相电感
#define LQ2 					(0.000925f)     // Q轴相电感
#define RS2 			        (0.916f/2) 	    // 定子相电阻
#define FI2 		            (0.01374f)	    // 磁链
#define ABZ_PULS2               (4096)          // ABZ模式式需要此参数

//motor 2
#define CurrentBase2   		(CURRENT_BASE1*10)      // 电流基值精度0.1A
#define VolatageBase2  		(53)  	                // 电压基值 精度1V，ADC能采到的最大值

#define OverCurrent2    		(140)  	    // 过流保护值
#define OverVoltage2    		(45) 		// 过压保护值
#define LackVoltage2    		(12) 		// 欠压保护值
#define ElectUpVoltage2 		(12) 		// 上电缓冲电压
#define AllowReset2	  			(0)			// 允许故障复位标志

#define DKp2  					(2500)	    // D轴KpQ13
#define DKi2	 				(1000)	    // D轴KiQ15
#define DOutMax2 				(15000)	    // D轴限幅Q15
#define QKp2  				    (2500)	    // Q轴KpQ13
#define QKi2  					(1000)	    // Q轴KiQ15
#define QOutMax2 				(15000)	    // Q轴限幅Q15	
#define SpdKi2			        (1000)	    // 转速环Ki//(5000)	
#define SpdKp2			        (2000)	    // 转速环Kp//(15000)
#define SpdOutMax2   			(10000)	    // 转速环输出限幅

/*****************转矩参数 *******************************/
//motor 1
#define MAX_DEF_TORQUE1         3000	    // 转矩控下的最高电流，Q15 Format	
#define STEP_TORQUE1            50
//motor 2
#define MAX_DEF_TORQUE2         3000	    // 转矩控下的最高电流，Q15 Format	
#define STEP_TORQUE2            50


/*****************end *******************************/
#endif

