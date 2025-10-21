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
 * @file data_uart.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _UART_DATA_H_
#define _UART_DATA_H_
#include "stdint.h"

#define DIS_FRAME_VAL 0x7e
#define ONE_FRAME_LEN 100

typedef struct
{
	uint32_t index;
	uint8_t Buf[ONE_FRAME_LEN];
	uint8_t State;
}RecvFrame;

enum
{
    MT_MOTORCHANGE = 0,
    MT_START,
    MT_STOP,
    MT_SPEED_SET,
    MT_SPEED_GET,
    MT_SPEED_PID_PARAM_SET,
    MT_SPEED_PID_PARAM_GET,
	MT_REG_VIEW_START,
	MT_REG_VIEW_STOP,
    MT_REG_VIEW_UPLOAD,
    
    //Mator Param
	MT_SET_MOTOR_PARAM = 0x50,
	MT_GET_MOTOR_PARAM,
	MT_START_TST_MOTOR_PARAM,
	MT_START_TST_BAK_MOTOR_PARAM,
	//HALL Param
	MT_SET_HALL_PARAM = 0x60,
	MT_GET_HALL_PARAM,
	//POS Parameter
	MT_SET_POS_PARAM = 0x65,
	MT_GET_POS_PARAM,
    //ENC Parameter
    MT_SET_ENC_PARAM = 0x6A,
    MT_GET_ENC_PARAM,
};


int32_t SubCharFill(uint8_t * pChar,int32_t len);



#endif

