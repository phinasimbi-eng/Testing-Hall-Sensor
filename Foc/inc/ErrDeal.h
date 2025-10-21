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
// 电机故障定义---------------------------------------------------------------
typedef union _MOTOR_FLAG_ALL_
{
    uint16_t Arr[2];
    struct
    {
			uint16_t SwOverCurrent :1;                // 软件过流
			uint16_t HwTotalErr    :1;                // 硬件故障
			uint16_t HwOverCurrent :1;                // 硬件过流
			uint16_t LowTempFlag	 :1;             		// 低温保护标志
			uint16_t HighTempFlag  :1;								// 高温保护标志
			uint16_t LackVolatage  :1;                // 欠压故障
			uint16_t OverVolatage  :1;                // 过压故障
			uint16_t IZeroFlag     :1;                // 零漂故障
			uint16_t Strcv0	    	 :8;
			
			uint16_t MotorStarted  :1;                // 电机运行
			uint16_t ShortPhase    :1;                // 三相短路标志
			uint16_t AllowStart    :1;                // 允许启动
			uint16_t MotorStartReq :1;                // 开机请求
			uint16_t ElectUpFlag   :1;                // 上电缓冲电压标志
			uint16_t DutyErr		   :1;                // 占空比计算错误
			uint16_t SystemReady   :1;                // 系统稳定标志
			uint16_t SlidingFlag   :1;                // 滑行检测标志
			uint16_t OverModulateFlag :1;             // 过调
			uint16_t ForceStartFlag  :1;							// 强制启动标志
			
			uint16_t BrakeFlag  :1;							// 强制启动标志
			uint16_t UpperControl  :1;							// 上位机启停控制
			uint16_t FOCControl  :1;							// 压缩机启停控制
			uint16_t Strcv    			: 3;
    } Bits;
} MotorFlag_Obj;



#endif 
