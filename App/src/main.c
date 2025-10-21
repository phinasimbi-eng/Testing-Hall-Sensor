/**
* @file main.c
* @author Nations Solution Team
* @version V1.0.0
* 
* @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
*/
#include "SystemDefine.h"

/*          old                     new
EN_BUT：  PF7(36pin)            PD13(36pin)
UART_TX:  TX1(46pin)(PB9)       TX4(18pin)(PB0) 
UART_RX:  RX1(13pin)(PA3)       RX4(19pin)(PB1)  
*/
// 系统主函数声明
int main(void);

// ========================================================================
// 函数名称：main
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：系统主函数
// ========================================================================
int main()
{
    System_Init();
    while(1)
    {
       MotorMain_Circle();
    }
}



















