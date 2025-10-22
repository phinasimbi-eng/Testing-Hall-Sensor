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
 * @file data_uart.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "MotorDrive.h"
#include "string.h"
#include "UserParam.h"
static int32_t SM_intcpy(void *dst, uint32_t val, uint32_t len);
static int32_t SM_intexp(uint8_t **pBuf, uint32_t len);
static int32_t SM_memcpy(void *dst, const void *src, uint32_t cnt);
static int32_t SM_strlen(const void *str);
extern uint8_t Fcs8(uint8_t *pVal, int32_t len);
extern int32_t AddCharFill(uint8_t *pChar, int32_t len);
extern int32_t SubCharFill(uint8_t *pChar, int32_t len);
extern void EncodePolarityRev(uint8_t num, uint8_t enable);
extern void DAC_update(uint8_t num, uint16_t data);
void Send_Data_DMA(uint8_t *data, const int len);

extern uint8_t SxBuffer1[BUFFSIZE];
float SM_intToFloat(u32 val);
void NzTransSend(uint8_t *pChar, int32_t len)
{
	pChar[len] = 0x7E;
	Send_Data_DMA(pChar, len); // DMA transmission
}

void Send_Data_DMA(uint8_t *data, const int len)
{
	memcpy(SxBuffer1, data, (len + 1));
	DMA_Channel_Disable(DMA_CH3);
	DMA_Current_Data_Transfer_Number_Set(DMA_CH3, (len + 1));
	DMA_Channel_Enable(DMA_CH3);
}

extern uint8_t HALLState;
extern uint8_t DMA_FLAG_TRANS_FINISHED;
void SendFixView_c(Motor_Obj *Obj, Motor_Obj *Obj1)
{
	static uint8_t SendBuf[BUFFSIZE]; //(n-1)*6=3000 n=501 500 data points Cnt=501 plus 2 bytes as header and tail 0x7E
	static int32_t SdLen = 0;
	static uint16_t Cnt = 0;
	if (Cnt < 1)
	{
		Cnt++;
		SdLen += SM_intcpy(SendBuf + SdLen, 0x7E, 1);
	}
	else if (Cnt < BUFFNUM + 1)
	{
		Cnt++;
#ifdef SEND_FLOAT
		SdLen += SM_intcpy(SendBuf + SdLen, *(unsigned long *)&Obj->DataObj.Ia_Phy, 4);
		SdLen += SM_intcpy(SendBuf + SdLen, *(unsigned long *)&Obj->DataObj.Ib_Phy, 4);
		SdLen += SM_intcpy(SendBuf + SdLen, *(unsigned long *)&Obj->DataObj.Ic_Phy, 4);
#endif
#ifdef SEND_INT
		// For Test
		SdLen += SM_intcpy(SendBuf + SdLen, Obj->SpeedObj.Speed_Target >> 3, 2);
		SdLen += SM_intcpy(SendBuf + SdLen, Obj->DataObj.MotorSpeed >> 3, 2);
		SdLen += SM_intcpy(SendBuf + SdLen, Obj->DataObj.IqObj, 2);
		SdLen += SM_intcpy(SendBuf + SdLen, Obj->CurLoop.IqFedBak, 2);
		SdLen += SM_intcpy(SendBuf + SdLen, Obj->DataObj.Ia_Q15, 2);
		SdLen += SM_intcpy(SendBuf + SdLen, Obj->DataObj.Ib_Q15, 2);
#endif
		if (SdLen > (BUFFSIZE - 2))
		{
			if (DMA_FLAG_TRANS_FINISHED == 1)
			{
				DMA_FLAG_TRANS_FINISHED = 0;
				NzTransSend(SendBuf, SdLen);
				SdLen = 0;
				Cnt = 0;
			}
		}
	}
	else
	{
	}
}

// ========================================================================
// Function name: OffsetValue_Save&Send
// Function: Save offset values for debugging
// Input parameters: None
// Input: None
// Output: None
// Function description: Print via UART & SPI for debugging, park offset values
// ========================================================================
#ifdef Offset_SEND
#define Num 2000	   // Storage data count
#define RefSpeed 50000 // Reference speed
#define TimeNum 60000  // Wait time to reach reference speed for storage 33.3us*60000 = 2s
int16_t OffsetValue[Num] = {0};
void OffsetValue_Save(Motor_Obj *Obj)
{
	static uint16_t Cnt = 0;
	static uint16_t TimeCnt = 0;

	if (Obj->Smo_data.SpeedRpm > RefSpeed)
	{
		if (TimeCnt < TimeNum)
			TimeCnt++;
		if (TimeCnt == TimeNum)
		{
			if (Cnt < Num)
				OffsetValue[Cnt++] = Obj->DataObj.Ib_Q15;
		}
	}
}

void OffsetValue_Send(Motor_Obj *Obj)
{
	static uint8_t Flag = 0;
	static uint16_t i = 0;
	static uint16_t j = 0;

	if (Obj->Smo_data.SpeedRpm > RefSpeed)
		Flag = 1;

	if (Obj->Flag.Bits.MotorStarted == 0 && Flag == 1)
	{
		for (i = 0; i < Num; i++)
		{
			printf("%d\r\n", OffsetValue[i]);
			// DAC_update(1,((OffsetValue[i]>>3)+2048));
		}
		if (i == Num)
		{
			for (j = 0; j < Num; j++)
			{
				OffsetValue[j] = 0;
			}
		}
		Flag = 0;
	}
}
#endif
// ========================================================================
// Function Name: ExploreParamRoot
// Description: Parameter exploration and query
// Input Parameters: Motor objects and data buffer
// Output Parameters: None
// Return Value: None
// Purpose: Parameter query and response function
// ========================================================================
float TargetSpeed = 0, SpeedKp = 2000, SpeedKi = 1000, CurDKp = 1200, CurDKi = 100, CurQKp = 1200, CurQKi = 100, PLLKp = _IQ(0.25), PLLKi = _IQ(3.25);
extern RecvFrame mRecvFrame;
void ExploreParamRoot_c(Motor_Obj *Obj, Motor_Obj *Obj1, uint8_t *pBuf, int32_t len)
{
	TargetSpeed = SM_intToFloat(SM_intexp(&pBuf, 4));
	SpeedKp = SM_intToFloat(SM_intexp(&pBuf, 4));
	SpeedKi = SM_intToFloat(SM_intexp(&pBuf, 4));
	CurDKp = SM_intToFloat(SM_intexp(&pBuf, 4));
	CurDKi = SM_intToFloat(SM_intexp(&pBuf, 4));
	CurQKp = SM_intToFloat(SM_intexp(&pBuf, 4));
	CurQKi = SM_intToFloat(SM_intexp(&pBuf, 4));
	PLLKp = SM_intToFloat(SM_intexp(&pBuf, 4));
	PLLKi = SM_intToFloat(SM_intexp(&pBuf, 4));
}

// ========================================================================
// Function Name: SM_intcpy
// Description: Integer copy with byte segmentation
// Input Parameters: Destination pointer, value, length
// Output Parameters: None
// Return Value: Status code
// Purpose: Copy integer data with 0x7e byte segmentation
// ========================================================================
int32_t SM_intcpy(void *dst, uint32_t val, uint32_t len)
{
	uint8_t *d = (uint8_t *)dst;
	uint32_t i;
	if (len <= 4)
	{
		for (i = 0; i < len; i++)
		{
			*(d + i) = val;
			val >>= 8;
		}
		return len;
	}
	return len;
}
float SM_intToFloat(u32 val)
{
	return *(float *)&val;
}
int32_t SM_intexp(uint8_t **pBuf, uint32_t len)
{
	int32_t rtn = 0;
	if (len <= 4)
	{
		SM_memcpy(&rtn, *pBuf, len);
		(*pBuf) += len;
	}
	return rtn;
}
int32_t SM_memcpy(void *dst, const void *src, uint32_t cnt)
{
	uint32_t rtn = cnt;
	uint8_t *d = (uint8_t *)dst;
	const uint8_t *s = (const uint8_t *)src;
	if (cnt == 0)
	{
		cnt = SM_strlen(src);
		rtn = cnt;
	}
	if (cnt)
	{
		do
			*d++ = *s++;
		while (--cnt);
	}
	return rtn;
}
int32_t SM_strlen(const void *str)
{
	const uint8_t *pstr = (const uint8_t *)str;
	const uint8_t *start = (const uint8_t *)str;
	while (*pstr)
		pstr++;
	return pstr - start;
}
