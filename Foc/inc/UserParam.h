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

#define HALL_FOR_ANGLE            //HALL�и�(1)		(ֻ��ͬʱ����(1)��(2)����Ч)
#define HALL_FOR_ANGLE_T          //HALL�и�(2)		(ֻ��ͬʱ����(1)��(2)����Ч)

/*********************ѡ����ģʽ*********************/
/* ע:����ת��ģʽ */

//#define TORQUE_CONTROL        //ת��ģʽ
#define SPEED_CONTROL           //�ٶ�ģʽ

///*********************��������ѡ�� *********************/
#define PCBA_SPEED_DEMO         //���ص�λ������

/*********************PCBA����*********************/
//motor 2
#define PWM_FREQUENCY2		10000l		    // �����Ƶ
#define CURRENT_BASE2 		(16.5f)			// ������ֵ
#define VOLTAGE_BASE2 	    (24.0f)			// ��ѹ��ֵ
#define LIMIT_DUTY2			30000l			// ռ�ձ������� max 32767l 

/*********************�������*********************/
//motor 2
#define MOTOR_POPAIRS2 			(2)			    // ���������
#define LD2 				    (0.000925f)     // D������
#define LQ2 					(0.000925f)     // Q������
#define RS2 			        (0.916f/2) 	    // ���������
#define FI2 		            (0.01374f)	    // ����
#define ABZ_PULS2               (4096)          // ABZģʽʽ��Ҫ�˲���

//motor 2
#define CurrentBase2   		(CURRENT_BASE1*10)      // ������ֵ����0.1A
#define VolatageBase2  		(53)  	                // ��ѹ��ֵ ����1V��ADC�ܲɵ������ֵ

#define OverCurrent2    		(140)  	    // ��������ֵ
#define OverVoltage2    		(45) 		// ��ѹ����ֵ
#define LackVoltage2    		(12) 		// Ƿѹ����ֵ
#define ElectUpVoltage2 		(12) 		// �ϵ绺���ѹ
#define AllowReset2	  			(0)			// ������ϸ�λ��־

#define DKp2  					(2500)	    // D��KpQ13
#define DKi2	 				(1000)	    // D��KiQ15
#define DOutMax2 				(15000)	    // D���޷�Q15
#define QKp2  				    (2500)	    // Q��KpQ13
#define QKi2  					(1000)	    // Q��KiQ15
#define QOutMax2 				(15000)	    // Q���޷�Q15	
#define SpdKi2			        (1000)	    // ת�ٻ�Ki//(5000)	
#define SpdKp2			        (2000)	    // ת�ٻ�Kp//(15000)
#define SpdOutMax2   			(10000)	    // ת�ٻ�����޷�

/*****************ת�ز��� *******************************/
//motor 1
#define MAX_DEF_TORQUE1         3000	    // ת�ؿ��µ���ߵ�����Q15 Format	
#define STEP_TORQUE1            50
//motor 2
#define MAX_DEF_TORQUE2         3000	    // ת�ؿ��µ���ߵ�����Q15 Format	
#define STEP_TORQUE2            50


/*****************end *******************************/
#endif

