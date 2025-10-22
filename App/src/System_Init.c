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
 * @file system_init.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "SystemDefine.h"
#include "SystemInterface.h"
#include "MotorDrive.h"
#include "string.h"

extern Motor_Obj motor_I[2];
extern uint8_t RxBuffer1[BUFFSIZE];
extern RecvFrame mRecvFrame;
extern void MotorPwm_Foc_Isr(uint8_t num, Motor_Obj *Obj, SystemInterface_Obj *InfObj);
// ϵͳ��ṹ������
static System_Obj SystemObj[2];
SystemInterface_Obj SystemInterfaceObj[2];

void System_Init(void);
static void Var_Init(void);
void MotorMain_Circle(void); // 2ms��ʱ����A
uint16_t ADC_ConvertedValue[2][ADC_CHANNEL_Number];
uint16_t *address_adc_value[2];

__IO uint16_t ADC_RegularConvertedValueTab[8];
__IO uint16_t ADC_InjectedConvertedValueTab[4];
__IO uint16_t PWM_CcSet[3];

// ========================================================================
// �������ƣ�System_Init
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������ϵͳ��ʼ��
// ========================================================================
void System_Init()
{
    FLASH_Latency_Set(FLASH_LATENCY_2);
    FLASH_Prefetch_Buffer_Enable();
    FLASH_ICache_Enable();

#define DGB_TIM1_STOP (1 << 10)
#define DGB_TIM8_STOP (1 << 17)
    uint32_t *p = (uint32_t *)0xE0042004;
    *p |= DGB_TIM1_STOP;
    *p |= DGB_TIM8_STOP;

    SystemClk_Init();
    __disable_irq();
    system_tick_init();
    Gpio_Init();
    Pwm_Init();
    Adc_Init();
#ifdef SPI_SEND
    TIM3_Init();
#endif
#ifdef UART_SEND
    Uart_init();
#endif
#ifdef SPI_SEND
    Spi_init();
#endif
    Var_Init();

    __enable_irq();
}

int16_t DoKeyEvent(uint16_t *incCnt, uint8_t *KeySate, uint16_t newKeyStatus)
{
    switch (*KeySate)
    {
    case 0:
        if (newKeyStatus == 1)
        {
            if ((*incCnt)++ > 300)
            {
                *incCnt = 0;
                *KeySate = 1;
            }
        }
        else
            *incCnt = 0;
        break;
    case 1:
        if (newKeyStatus == 0)
        {
            if ((*incCnt)++ > 300)
            {
                *incCnt = 0;
                *KeySate = 0;
                return 1;
            }
        }
        else
            *incCnt = 0;
        break;
    }
    return 0;
}
int16_t GetKeyEventStart(void)
{
    static uint16_t incCnt = 0;
    static uint8_t KeySate = 0;
    uint16_t newKeyStatus = GetIO_Button1();
    return DoKeyEvent(&incCnt, &KeySate, newKeyStatus);
}
// ========================================================================
// �������ƣ�SpeedGiven
// �����������
// �����������
// ��    �룺��
// ��    ������
// �����������ٶȸ�������
// ========================================================================
void SpeedGiven(void)
{
    int32_t Speed_Target = SystemObj[0].SpeedValue * MTR_VOLT_RPM;
    static int32_t SpeedFilter = 0;
    SpeedFilter = Speed_Target;
    // �ٶȸ�ֵ

    if (SpeedFilter < MTR_MINRPM)
    {
        SpeedFilter = MTR_MINRPM;
    }

    if (SpeedFilter > MTR_MAXRPM)
    {
        SpeedFilter = MTR_MAXRPM;
    }

    SpeedControl(0, SpeedFilter);
}
// ========================================================================
// �������ƣ�ControlSpeed
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������BSP�û��������������Ƶ����
// ========================================================================
void ControlSpeed(void)
{
    static uint8_t StopOrStop = 0;

    if (GetKeyEventStart())
    {
        if (StopOrStop == 1)
        {
            StopOrStop = 2;
            LEDON_Control(LED_WORK);
            StartStopControl(0, 1);
        }
        else if (StopOrStop == 2)
        {
            StopOrStop = 1;
            LEDOFF_Control(LED_WORK);
            StartStopControl(0, 0);
        }
    }
    if (StopOrStop == 0)
    {
        StopOrStop = 1;
        LEDOFF_Control(LED_WORK);
        LEDOFF_Control(LED_VOLTAGE_ERR);
        LEDOFF_Control(LED_CURRENT_ERR);
    }
}

// ========================================================================
// �������ƣ�delay_decrement
// �����������
// �����������
// ��    �룺��
// ��    ������
// ������������ʱ����
// ========================================================================
static volatile uint32_t delay;
void delay_decrement(void);
void delay_100us(uint32_t count);
void delay_decrement(void)
{
    if (0U != delay)
    {
        delay--;
    }
}
void delay_100us(uint32_t count)
{
    delay = count;

    while (0U != delay)
    {
    }
}
// ========================================================================
// �������ƣ�SysTick_Handler
// �����������
// �����������
// ��    �룺��
// ��    ������
// �����������δ�ʱ���жϣ�������Ƶ����
// ========================================================================
void SysTick_Handler(void)
{
    static uint16_t freq_divde = 0;

    freq_divde++;
    if ((freq_divde % 2) == 1)
    {
        MotorIsr_200us(0, &SystemInterfaceObj[0]);
    }

#ifdef PCBA_SPEED_DEMO
    ControlSpeed();
#endif
    delay_decrement();
}
// ========================================================================
// �������ƣ�DMA_Channel4_IRQHandler
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������DMA�жϺ���
// ========================================================================
void DMA_Channel4_IRQHandler(void)
{
    if (SET == DMA_Interrupt_Status_Get(DMA, DMA_CH4_INT_TXC))
    {
        DMA_Interrupt_Status_Clear(DMA, DMA_CH4_INT_TXC);
    }
    memcpy(mRecvFrame.Buf, RxBuffer1, BUFFSIZE1);
    for (uint8_t i = 0; i < BUFFSIZE1; i++)
    {
        Uart_Isr(mRecvFrame.Buf[i]);
    }
    /*���·���һ�����ݴ���*/
    DMA_Channel_Disable(DMA_CH4);
    DMA_Current_Data_Transfer_Number_Set(DMA_CH4, BUFFSIZE1);
    DMA_Channel_Enable(DMA_CH4);
}
// ========================================================================
// �������ƣ�DMA_Channel3_IRQHandler
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������DMA�жϺ���
// ========================================================================
uint8_t DMA_FLAG_TRANS_FINISHED = 1;
void DMA_Channel3_IRQHandler(void)
{
    if (SET == DMA_Interrupt_Status_Get(DMA, DMA_CH3_INT_TXC))
    {
        DMA_Interrupt_Status_Clear(DMA, DMA_CH3_INT_TXC);
        DMA_FLAG_TRANS_FINISHED = 1;
    }
}
// ========================================================================
// �������ƣ�ADC1_2_IRQHandler
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������ADC1-2�ɼ���������ж�
// ========================================================================
void ADC_IRQHandler(void)
{
    // delay_decrement();
    // TestGpioHigh();
    if (ADC_INTFlag_Status_Get(ADC_INT_FLAG_JENDC) == SET)
    {
        ADC_INTFlag_Status_Clear(ADC_INT_FLAG_JENDC);

        ADC_ConvertedValue[0][0] = 0;
        ADC_ConvertedValue[0][1] = ADC_Injected_Group_Conversion_Data_Get(ADC_INJECTED_DATA_REG_1); // v1
        ADC_ConvertedValue[0][2] = ADC_Injected_Group_Conversion_Data_Get(ADC_INJECTED_DATA_REG_2); // w1
        ADC_ConvertedValue[0][3] = 0x05a8;                                                          // ADC_RegularConvertedValueTab[1];// vbus
        ADC_ConvertedValue[0][4] = 500;
        SystemObj[0].VoltageValue = ADC_RegularConvertedValueTab[1];
        SystemObj[0].SpeedValue = ADC_RegularConvertedValueTab[0];

        ADC_InjectedConvertedValueTab[0] = ADC_ConvertedValue[0][1];
        ADC_InjectedConvertedValueTab[1] = ADC_ConvertedValue[0][2];
        ADC_InjectedConvertedValueTab[2] = ADC_Injected_Group_Conversion_Data_Get(ADC_INJECTED_DATA_REG_3);

        ADC_Regular_Channels_Software_Conversion_Operation(ADC_EXTRTRIG_SWSTRRCH_ENABLE);

#ifdef HALL_FOR_ANGLE_T // ����FOC
        MotorPwm_Isr_I(0, &SystemInterfaceObj[0]);
#endif
    }
    else
    {
        if (ADC_INTFlag_Status_Get(ADC_INT_FLAG_AWDG) == SET)
            ADC_INTFlag_Status_Clear(ADC_INT_FLAG_AWDG);
    }
    // TestGpioLow();
}

// ========================================================================
// �������ƣ�TIM1_BRK_IRQHandler
// �����������
// �����������
// ��    �룺��
// ��    ������
// ��������������ɲ���жϺ���
// ========================================================================
void TIM1_BRK_IRQHandler()
{
    if (TIM_Interrupt_Status_Get(TIM1, TIM_STS_BITF) != RESET)
    {
        TIM_Interrupt_Status_Clear(TIM1, TIM_STS_BITF);
        BrakeErrIsr_I(0);
    }
}
// ========================================================================
// �������ƣ�COMP1_2_3_IRQHandler
// �����������
// �����������
// ��    �룺��
// ��    ������
// �����������Ƚ����жϺ���
// ========================================================================
void COMP_1_2_3_IRQHandler()
{
    if (EXTI_Interrupt_Status_Get(EXTI_LINE18))
    {
        EXTI_Interrupt_Status_Clear(EXTI_LINE18);
    }
    if (COMP_Interrupt_Status_Get())
    {
        COMP_Interrupt_Status_OneComp_Clear(COMP1);
        BrakeErrIsr_I(0);
    }
}
// ========================================================================
// �������ƣ�Var_Init
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������������ʼ��
// ========================================================================
void Var_Init()
{
    address_adc_value[0] = ADC_ConvertedValue[0];
    address_adc_value[1] = ADC_ConvertedValue[1];
    /******************** motor1���ݲ�����ʼ��***********************/
    // ����
    SystemInterfaceObj[0].IaSample_I = address_adc_value[0];
    SystemInterfaceObj[0].IbSample_I = (address_adc_value[0] + 1);
    SystemInterfaceObj[0].IcSample_I = (address_adc_value[0] + 2);
    // ��ѹ
    SystemInterfaceObj[0].UdcSample_I = (address_adc_value[0] + 3);
    // ������ֵ
    SystemInterfaceObj[0].Half_duty_I = &SystemObj[0].Half_duty;

    SystemObj[0].IaSample = 0;
    SystemObj[0].IbSample = 0;
    SystemObj[0].IcSample = 0;
    SystemObj[0].UdcSample = 0;
    SystemObj[0].Half_duty = HALFDUTY_1;

    /********************* motor2���ݲ�����ʼ��**********************/
    // ����
    SystemInterfaceObj[1].IaSample_I = address_adc_value[1];
    SystemInterfaceObj[1].IbSample_I = (address_adc_value[1] + 1);
    SystemInterfaceObj[1].IcSample_I = (address_adc_value[1] + 2);
    // ��ѹ
    SystemInterfaceObj[1].UdcSample_I = (address_adc_value[1] + 3);
    // ������ֵ
    SystemInterfaceObj[1].Half_duty_I = &SystemObj[1].Half_duty;

    SystemObj[1].IaSample = 0;
    SystemObj[1].IbSample = 0;
    SystemObj[1].IcSample = 0;
    SystemObj[1].UdcSample = 0;
    SystemObj[1].Half_duty = HALFDUTY_8;

    MotorDrive_Init(&SystemInterfaceObj[0], &SystemInterfaceObj[1]);
}
// ========================================================================
// �������ƣ�MotorMain_Circle
// �����������
// �����������
// ��    �룺��
// ��    ������
// ������������ѭ����������
// ========================================================================
void MotorMain_Circle()
{
    ForMotorMain(&SystemInterfaceObj[0], &SystemInterfaceObj[1]);
}
