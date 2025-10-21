/**
* @file main.c
* @author Nations Solution Team
* @version V1.0.0
* 
* @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
*/
#include "SystemDefine.h"

/*          old                     new
EN_BUT��  PF7(36pin)            PD13(36pin)
UART_TX:  TX1(46pin)(PB9)       TX4(18pin)(PB0) 
UART_RX:  RX1(13pin)(PA3)       RX4(19pin)(PB1)  
*/
// ϵͳ����������
int main(void);

// ========================================================================
// �������ƣ�main
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������ϵͳ������
// ========================================================================
int main()
{
    System_Init();
    while(1)
    {
       MotorMain_Circle();
    }
}



















