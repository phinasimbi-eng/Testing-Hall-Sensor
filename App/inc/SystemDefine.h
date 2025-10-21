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
 * @file systemdefine.h
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _SYSTEMDEFINE_H_
#define _SYSTEMDEFINE_H_


#include "n32g430.h"

#include "SystemInterface.h"

typedef struct SYSTEMINTERFACE_OBJ
{
	// ADC数据
	uint16_t IaSample;   // A相电流采样
	uint16_t IbSample;   // B相电流采样
	uint16_t IcSample;   // C相电流采样
	uint16_t UdcSample;  // 母线电压采样
	uint16_t Half_duty;
    uint16_t SpeedValue;
    uint16_t VoltageValue;
}System_Obj;

#define HALFDUTY_1          (TIM1->AR >> 1)
#define HALFDUTY_8          (TIM8->AR >> 1)

#define ADC_CHANNEL_Number 								7

#define LED_ON			    GPIO_ResetBits(GPIOC,GPIO_PIN_9);   // 拉低
#define LED_OFF			    GPIO_SetBits(GPIOC,GPIO_PIN_9);     // 拉高

#define BIT0     ((uint32_t)1 << 0)
#define BIT1     ((uint32_t)1 << 1)
#define BIT2     ((uint32_t)1 << 2)
#define BIT3     ((uint32_t)1 << 3)
#define BIT4     ((uint32_t)1 << 4)
#define BIT5     ((uint32_t)1 << 5)
#define BIT6     ((uint32_t)1 << 6)
#define BIT7     ((uint32_t)1 << 7)
#define BIT8     ((uint32_t)1 << 8)
#define BIT9     ((uint32_t)1 << 9)
#define BIT10    ((uint32_t)1 << 10)
#define BIT11    ((uint32_t)1 << 11)
#define BIT12    ((uint32_t)1 << 12)
#define BIT13    ((uint32_t)1 << 13)
#define BIT14    ((uint32_t)1 << 14)
#define BIT15    ((uint32_t)1 << 15)
#define BIT16    ((uint32_t)1 << 16)
#define BIT17    ((uint32_t)1 << 17)
#define BIT18    ((uint32_t)1 << 18)
#define BIT19    ((uint32_t)1 << 19)
#define BIT20    ((uint32_t)1 << 20)
#define BIT21    ((uint32_t)1 << 21)
#define BIT22    ((uint32_t)1 << 22)
#define BIT23    ((uint32_t)1 << 23)
#define BIT24    ((uint32_t)1 << 24)
#define BIT25    ((uint32_t)1 << 25)
#define BIT26    ((uint32_t)1 << 26)
#define BIT27    ((uint32_t)1 << 27)
#define BIT28    ((uint32_t)1 << 28)
#define BIT29    ((uint32_t)1 << 29)
#define BIT30    ((uint32_t)1 << 30)
#define BIT31    ((uint32_t)1 << 31)

//#if 0 //by 032
//#define MY_UART_BOARTE					(115200l)
//#define MY_UART                 UART5
//#define MY_UART_IRQn            UART5_6_IRQn
//#define MY_UART_IRQHandler      UART5_6_IRQHandler
//#else //by 030
//#define MY_UART_BOARTE					(115200l)
//#define MY_UART                 USART1
//#define MY_UART_IRQn            USART1_IRQn
//#define MY_UART_IRQHandler      USART1_IRQHandler
//#endif

typedef enum
{
    LED_WORK = 1,
    LED_VOLTAGE_ERR,
    LED_CURRENT_ERR,
}LedCtrl;

// 外部函数声明
extern void HAL_NvicConfiguration(void);
extern void System_Init(void); 			// 系统初始化函数
extern void SystemClk_Init(void);       // 系统时钟初始化
extern void Gpio_Init(void);            // pwm IO初始化
extern void Adc_Init(void);             // Adc初始化
extern void system_tick_init(void);
extern void Pwm_Init(void);             // PWM初始化
extern void Pwm_II_Init(void);
extern void TIM3_Init(void);
extern void Uart_init(void);
extern void Spi_init(void);
extern void EncodePolarityRev(uint8_t num,uint8_t enable);
extern uint16_t GetIO_GetEnc(uint8_t num);
extern void GetIO_ResetEnc(uint8_t num,uint16_t cnt);
// 内部调用定时函数
extern void MotorMain_Circle(void);
extern void Step_motor_1ms(void);
extern void LEDON_Control(LedCtrl mLed);
extern void LEDOFF_Control(LedCtrl mLed);
extern void TestGpioHigh(void);
extern void TestGpioLow(void);
extern void delay_100us(uint32_t count);
extern void TIM3_Delay_Us(__IO uint32_t us);
#endif  
