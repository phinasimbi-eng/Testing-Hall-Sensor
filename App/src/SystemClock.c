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
 * @file systemclock.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "SystemDefine.h"
#include "UserParam.h"

void HAL_NvicConfiguration(void);
void SystemClk_Init(void);  // 系统时钟初始化
void Gpio_Init(void);       // GPIO初始化
void system_tick_init(void);
uint16_t GetIO_Button1(void);  // 获取启动按键状态
void LEDON_Control(LedCtrl mLed);
void LEDOFF_Control(LedCtrl mLed);
// ========================================================================
// 函数名称：SystemClk_Init
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：系统时钟初始化
// ========================================================================
void SystemClk_Init()
{
    // Enable peripheral clocks
    // Enable TIM1 clocks 
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_TIM1);
    // Enable DMA clocks 
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA);

    // Enable GPIOC clocks 
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA | RCC_AHB_PERIPH_GPIOB | RCC_AHB_PERIPH_GPIOC | RCC_AHB_PERIPH_GPIOD );
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    // Enable ADC clocks 
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_ADC);

    // RCC_ADCHCLK_DIVx,max 18MHz,48/4 = 12
    ADC_Clock_Mode_Config(ADC_CKMOD_AHB, RCC_ADCHCLK_DIV2);
    RCC_ADC_1M_Clock_Config(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8);
    
    // Enable COMP clocks 
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_COMP | RCC_APB1_PERIPH_COMP_FILT);
    
		//UART  clocks enable 
		RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_UART3);
  
	  /* PCLK2 = HCLK/2 */
    RCC_Pclk2_Config(RCC_HCLK_DIV2);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_SPI2);
		
    //TIM3
		RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_TIM3);	
}

// ========================================================================
// 函数名称：GetIO_GetEnc
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：abz Enc 初始化
// ========================================================================
TIM_Module* TIMSz[]={TIM3};
uint16_t GetIO_GetEnc(uint8_t num)
{
    uint16_t tmp = TIMSz[num]->CNT;
    return tmp;
}
void GetIO_ResetEnc(uint8_t num,uint16_t cnt)
{
    TIMSz[num]->CNT = cnt;
}

void EncodePolarityRev(uint8_t num,uint8_t enable)
{
    TIM_Output_Channel_Polarity_Set(TIMSz[num],enable?TIM_OC_POLARITY_LOW:TIM_OC_POLARITY_HIGH,TIM_CH_1);
}


void TIM3_Init(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    TIM_Base_Struct_Initialize(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period    = 2-1;
    TIM_TimeBaseStructure.Prescaler = 32-1;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;

    TIM_Base_Initialize(TIM3, &TIM_TimeBaseStructure);  
}

void TIM3_Delay_Us(__IO uint32_t us)
{
    TIM_Base_Count_Set(TIM3,0);
    TIM_On(TIM3);
    while(TIM_Base_Count_Get(TIM3) < us);
    TIM_Off(TIM3);
}

void Gpio_Init(void)
{
	GPIO_InitType GPIO_InitStructure;

    // Configure PC.06 as output push-pull,work status
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT_PP;
    GPIO_InitStructure.Pin       = GPIO_PIN_2|GPIO_PIN_7;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.Pin       = GPIO_PIN_15;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    //Test Gpio out
    GPIO_InitStructure.Pin       = GPIO_PIN_14;
    GPIO_Peripheral_Initialize(GPIOC, &GPIO_InitStructure);
    //Button
    GPIO_InitStructure.Pin       = GPIO_PIN_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_INPUT;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);

		// Configure TIM1, CH1(PA8),CH2(PA9),CH3(PA10),CH4() as alternate function push-pull
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
		GPIO_InitStructure.GPIO_Alternate = GPIO_AF3_TIM1;
    GPIO_InitStructure.Pin        = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Alternate = GPIO_AF3_TIM1;
    GPIO_InitStructure.Pin        = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
	
	
		//adc configure
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
    //current
    GPIO_InitStructure.Pin       = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;//V,W,U
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    //other adc
    GPIO_InitStructure.Pin       = GPIO_PIN_0;//speed
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.Pin       = GPIO_PIN_3;//vbus
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    
    //comp PA12 comp1 inp
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
	
		#ifdef UART_SEND	
	GPIO_InitStructure.GPIO_Current = GPIO_DS_12MA;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF10_UART3;
	GPIO_InitStructure.Pin = GPIO_PIN_10;//TX3
	GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF10_UART3;
	GPIO_InitStructure.Pin = GPIO_PIN_11;//RX3
	GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);	
		#endif
	
		#ifdef SPI_SEND	
  GPIO_Structure_Initialize(&GPIO_InitStructure);		
	GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
	GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;

	GPIO_InitStructure.GPIO_Alternate = GPIO_AF1_SPI2;
	GPIO_InitStructure.Pin        = GPIO_PIN_12;
	GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF1_SPI2;
	GPIO_InitStructure.Pin        = GPIO_PIN_12;
	GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
		
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_SPI2;
	GPIO_InitStructure.Pin        = GPIO_PIN_12;
	GPIO_Peripheral_Initialize(GPIOD, &GPIO_InitStructure);		
		#endif		
		
    #ifdef HALL_FOR_ANGLE
	GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
	GPIO_InitStructure.Pin = GPIO_PIN_6|GPIO_PIN_4|GPIO_PIN_5;
	GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure); 
		#endif	
}

// ========================================================================
// 函数名称：GetIO_GetHall
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：Get Hall State
// ========================================================================
uint8_t GetIO_GetHall(uint8_t num)
{
	uint8_t KeyState = GPIO_Input_Data_Get(GPIOB);
	if(num == 0)
    {
        static uint8_t cnt = 0;
        static uint8_t PreKeyState;
        KeyState = ((KeyState&BIT5)>>3)+((KeyState&BIT4)>>3)+((KeyState&BIT6)>>6);
        
        if(PreKeyState != KeyState)
        {
            cnt++;
            if(cnt > 3)
            {
                cnt = 0;
                PreKeyState = KeyState;
            }
        }
        else
            cnt = 0;
        return PreKeyState;
    }
	return 0;
}
// ========================================================================
// 函数名称：GetIO_Button1
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：获取启动按键状态
// ========================================================================
uint16_t GetIO_Button1(void)
{
	uint16_t KeyState = 0;
	KeyState = GPIO_Input_Pin_Data_Get(GPIOB, GPIO_PIN_3);
	return KeyState;
}
// ========================================================================
// 函数名称：system_tick_init
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：用于执行低频任务
// ========================================================================
void system_tick_init(void)
{
	SysTick_Config(MAIN_FREQUENCY/10000);
	SysTick->CTRL |= 0x00000004U;
	NVIC_SetPriority(SysTick_IRQn,2);
}
// ========================================================================
// 函数名称：LEDON_Control
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：LED开
// ========================================================================
void LEDON_Control(LedCtrl mLed)
{
    if(mLed == LED_WORK)
        GPIO_PBC_Pins_Reset(GPIOB,GPIO_PIN_2);
    if(mLed == LED_VOLTAGE_ERR)
        GPIO_PBC_Pins_Reset(GPIOB,GPIO_PIN_7);
    if(mLed == LED_CURRENT_ERR)
        GPIO_PBC_Pins_Reset(GPIOA,GPIO_PIN_15);
}
// ========================================================================
// 函数名称：LEDOFF_Control
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：LED关
// ========================================================================
void LEDOFF_Control(LedCtrl mLed)
{
    if(mLed == LED_WORK)
        GPIO_Pins_Set(GPIOB,GPIO_PIN_2);
    if(mLed == LED_VOLTAGE_ERR)
        GPIO_Pins_Set(GPIOB,GPIO_PIN_7);
    if(mLed == LED_CURRENT_ERR)
        GPIO_Pins_Set(GPIOA,GPIO_PIN_15);
}
void TestGpioHigh(void)
{
    GPIO_Pins_Set(GPIOC,GPIO_PIN_14);
}
void TestGpioLow(void)
{
    GPIO_PBC_Pins_Reset(GPIOC,GPIO_PIN_14);
}
