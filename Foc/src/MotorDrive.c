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
 * @file motordrive.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "MotorDrive.h"
#include "UserParam.h"

Motor_Obj motor_I[2];
RecvFrame mRecvFrame = {0};
/***********************外部接口函数*******************************/
void MotorDrive_Init(SystemInterface_Obj *InfObj,SystemInterface_Obj *InfObj1); // 电机层参数初始化
void ForMotorMain(SystemInterface_Obj *InfObj,SystemInterface_Obj *InfObj1);
void MotorIsr_200us(uint8_t num,SystemInterface_Obj *InfObj);  // 电机层100us中断函数
void Uart_Isr(uint8_t data_in);
void MotorPwm_Isr_I(uint8_t num,SystemInterface_Obj *InfObj);    // 电机层PWM中断函数
void BrakeErrIsr_I(uint8_t num);

/*********************** 内部接口函数*******************************/
// 初始化函数
void SvpwmInit(Motor_Obj *Obj,uint16_t *Half_duty_I);
void SvpwmInit_c(Motor_Obj *Obj,uint16_t *Half_duty_I);
void ErrDealInit(Motor_Obj *Obj); // 故障初始化函数
void DataDealInit(Motor_Obj *Obj);
void DataDealInit_c(Motor_Obj *Obj);
void CurLoopInit(Motor_Obj *Obj);
void CurLoopInit_c(Motor_Obj *Obj);
void BrushlessInit(Motor_Obj *Obj);
void SpeedCtrlInit(uint8_t num,Motor_Obj *Obj);
void Foc_para_init(Motor_Obj *Obj,Motor_Obj *Obj1);

// 算法计算函数
void SvpwmNew_Isr(uint8_t num,Motor_Obj *Obj,int32_t UdOut_Q15,int32_t UqOut_Q15);
void SvpwmNew_Isr_c(uint8_t num,Motor_Obj *Obj,int32_t UdOut_Q15,int32_t UqOut_Q15);
void StartStop_Isr(Motor_Obj *Obj,uint8_t num);   // 起停控制
void IdIqFbCalcu(Motor_Obj *Obj);     // 反馈电流计算
void IdIqFbCalcu_c(Motor_Obj *Obj);     // 反馈电流计算
void CurrentQ15Calcu(Motor_Obj *Obj,SystemInterface_Obj *InfObj);
void CurrentQ15Calcu_c(Motor_Obj *Obj,SystemInterface_Obj *InfObj);
void BrushlessPiCtrl(uint8_t num,Motor_Obj *Obj);
void ElectUpDected(Motor_Obj *Obj);
void ElectUpDected_c(Motor_Obj *Obj);
void Task_Deapare(Motor_Obj *Obj);
void Speed_air_2ms(uint8_t num,Motor_Obj *Obj);
void Speed_air_2ms_IF(uint8_t num,Motor_Obj *Obj);
void Fluxwkn_Run(Motor_Obj *Obj);
void Fluxwkn_Run1(Motor_Obj *Obj);
int16_t SmoForceStartCtrl(Motor_Obj *Obj,uint8_t num);
int8_t GetHallLine_c(HallCalc_t * pHallCalc,int8_t HallIo);
void Observer_Init(Motor_Obj *Obj);
int16_t	Observer_Run(uint8_t num,Motor_Obj *Obj);
void SpeedLoop(uint8_t num,Motor_Obj *Obj);
	
void IFAngle_init(Motor_Obj *Obj);			//IF参数初始化

// 定时中断处理函数
void ErrDeal_Isr(Motor_Obj *Obj);       // 故障处理函数
void IPhysicalCalcu_100us(Motor_Obj *Obj); // 电流有效值和最大值计算
void IPhysicalCalcu_100us_c(Motor_Obj *Obj); // 电流有效值和最大值计算
void IzeroReCalc(Motor_Obj *Obj,SystemInterface_Obj *InfObj);
void IzeroReCalc_c(Motor_Obj *Obj,SystemInterface_Obj *InfObj);
void SpeedGiven(void);

// 串口通信
void ExploreParamRoot(Motor_Obj *Obj,Motor_Obj *Obj1,uint8_t*pBuf,int32_t len);
void ExploreParamRoot_c(Motor_Obj *Obj,Motor_Obj *Obj1,uint8_t*pBuf,int32_t len);
void SendFixView(Motor_Obj *Obj,Motor_Obj *Obj1);
void SendFixView_c(Motor_Obj *Obj,Motor_Obj *Obj1);
void OffsetValue_Send(Motor_Obj *Obj);
	
// SPI通信
extern void DAC_update(uint8_t num,uint16_t data);

// 主循环函数
void Uart_Deal(Motor_Obj *Obj,Motor_Obj *Obj1);
void Spi_Deal(Motor_Obj *Obj,Motor_Obj *Obj1);
void OLED_Deal(Motor_Obj *Obj,Motor_Obj *Obj1);
	
void SpeedControl(uint8_t num,int32_t TargetSpeed)
{
    motor_I[num].SpeedObj.Speed_Target = TargetSpeed;
}
void StartStopControl(uint8_t num,uint16_t RunEn)
{
    motor_I[num].SoftStartStopCtrl = RunEn;
}
// ========================================================================
// 函数名称：MotorDrive_Init
// 输入参数：无
// 扇    入：无
// 输出参数：无
// 扇    出：无
// 函数描述：电机层参数初始化
// ========================================================================
void MotorDrive_Init(SystemInterface_Obj *InfObj,SystemInterface_Obj *InfObj1)
{
/***************motor controller 1 parameters  ******************/
        motor_I[0].Plate.Motor_pole = MOTOR_POPAIRS1;
        motor_I[0].Plate.Pwm_freq = PWM_FREQUENCY1;
		motor_I[0].DelayCnt = 0;
		motor_I[0].SoftStartStopCtrl = 0;
		motor_I[0].SystemCount = 0;
		motor_I[0].ClearOnce = 0;
		motor_I[0].MotorState = STOP_STATE;
		motor_I[0].MotorBaseObj.CurrentBase = CurrentBase1;         // 1A->0.02V 1.65V,保护作用
		motor_I[0].MotorBaseObj.VolatageBase = VolatageBase1;       // 电压基值 48V->1.84V
		motor_I[0].MotorBaseObj.OverCurrent = OverCurrent1;         // 过流保护值 电流精度0.1A
		motor_I[0].MotorBaseObj.OverVoltage = OverVoltage1;         // 过压保护值
		motor_I[0].MotorBaseObj.LackVoltage = LackVoltage1;         // 欠压保护值
		motor_I[0].MotorBaseObj.ElectUpVoltage = ElectUpVoltage1;   // 上电缓冲电压35V
        
		motor_I[0].MotorBaseObj.AllowReset = AllowReset1;           // 允许故障复位标志
		motor_I[0].MotorBaseObj.Fun_DKp_Q13 = DKp1;                 // D环Kp 1500
		motor_I[0].MotorBaseObj.Fun_DKi_Q15 = DKi1;                 // D环Ki 1000
		motor_I[0].MotorBaseObj.Fun_DOutMax_Q15 = DOutMax1;         // D环限幅
		motor_I[0].MotorBaseObj.Fun_QKp_Q13 = QKp1;                 // Q环Kp
		motor_I[0].MotorBaseObj.Fun_QKi_Q15 = QKi1;                 // Q环Ki
		motor_I[0].MotorBaseObj.Fun_QOutMax_Q15 = QOutMax1;         // Q环限幅
		motor_I[0].MotorBaseObj.Fun_SpdOutMax = SpdOutMax1;         // Spd环限幅
		motor_I[0].MotorBaseObj.Fun_SpdKi = SpdKi1;                 // Spd环Ki 400
		motor_I[0].MotorBaseObj.Fun_SpdKp = SpdKp1;                 // Spd环Kp 2000	
        
		motor_I[0].SpeedObj.Speed_Target = 500;
		motor_I[0].SpeedObj.target_calc = 0;
		motor_I[0].SpeedObj.step_target = 20;
/***************motor controller 2 parameters  ******************/
        motor_I[1].Plate.Motor_pole = MOTOR_POPAIRS2;
        motor_I[1].Plate.Pwm_freq = PWM_FREQUENCY2;
		motor_I[1].DelayCnt = 0;
		motor_I[1].SoftStartStopCtrl = 0;
		motor_I[1].SystemCount = 0;
		motor_I[1].ClearOnce = 0;
		motor_I[1].MotorState = STOP_STATE;
		motor_I[1].MotorBaseObj.CurrentBase = CurrentBase2;         // 1A->0.02V 1.65V
		motor_I[1].MotorBaseObj.VolatageBase = VolatageBase2;       // 电压基值 48V->1.84V
		motor_I[1].MotorBaseObj.OverCurrent = OverCurrent2;         // 过流保护值 电流精度0.1A
		motor_I[1].MotorBaseObj.OverVoltage = OverVoltage2;         // 过压保护值
		motor_I[1].MotorBaseObj.LackVoltage = LackVoltage2;         // 欠压保护值
		motor_I[1].MotorBaseObj.ElectUpVoltage = ElectUpVoltage2;   // 上电缓冲电压35V
		
		motor_I[1].MotorBaseObj.AllowReset = AllowReset2;           //0;
		motor_I[1].MotorBaseObj.Fun_DKp_Q13 = DKp2;                 // D环Kp 1500
		motor_I[1].MotorBaseObj.Fun_DKi_Q15 = DKi2;                 // D环Ki 1000
		motor_I[1].MotorBaseObj.Fun_DOutMax_Q15 = QOutMax2;         // D环限幅
		motor_I[1].MotorBaseObj.Fun_QKp_Q13 = QKp2;                 // Q环Kp
		motor_I[1].MotorBaseObj.Fun_QKi_Q15 = QKi2;                 // Q环Ki
		motor_I[1].MotorBaseObj.Fun_QOutMax_Q15 = QOutMax2;         // Q环限幅
		motor_I[1].MotorBaseObj.Fun_SpdOutMax = SpdOutMax2;         // Spd环限幅
		motor_I[1].MotorBaseObj.Fun_SpdKi = SpdKi2;                 // Spd环Ki 400
		motor_I[1].MotorBaseObj.Fun_SpdKp = SpdKp2;                 // Spd环Kp 2000
        
		motor_I[1].SpeedObj.Speed_Target = 500;
		motor_I[1].SpeedObj.target_calc = 0;
		motor_I[1].SpeedObj.step_target = 10;
/***************motor 1 & motor 2 initial*****************/			
		SvpwmInit_c(&motor_I[0],InfObj->Half_duty_I);		
		SvpwmInit_c(&motor_I[1],InfObj->Half_duty_I);	
		DataDealInit_c(&motor_I[0]);	
		DataDealInit_c(&motor_I[1]);					
		CurLoopInit_c(&motor_I[0]);			
		CurLoopInit_c(&motor_I[1]);			
		ErrDealInit(&motor_I[0]);
		ErrDealInit(&motor_I[1]);
		BrushlessInit(&motor_I[0]);
		BrushlessInit(&motor_I[1]);
		SpeedCtrlInit(0,&motor_I[0]);
		SpeedCtrlInit(1,&motor_I[1]);
		Foc_para_init(&motor_I[0],&motor_I[1]);			
}
// ========================================================================
// 函数名称：Foc_para_init_I
// 输入参数：无
// 扇    入：无
// 输出参数：无
// 扇    出：无
// 函数描述：FOC算法参数初始化
// ========================================================================
void Foc_para_init(Motor_Obj *Obj,Motor_Obj *Obj1)
{
    #ifdef HALL_FOR_ANGLE
        /***************	motor 1 parameters  ******************/
        Obj->HallCalc.Pwm_freq = PWM_FREQUENCY1;
        Obj->HallCalc.Pols = MOTOR_POPAIRS1;
        //state parameter
        Obj->HallCalc.HallFixLineSeq[0] = 3;
        Obj->HallCalc.HallFixLineSeq[1] = 1;
        Obj->HallCalc.HallFixLineSeq[2] = 5;
        Obj->HallCalc.HallFixLineSeq[3] = 4;
        Obj->HallCalc.HallFixLineSeq[4] = 6;
        Obj->HallCalc.HallFixLineSeq[5] = 2;
        Obj->HallCalc.HallFixAngle[0] = 24334;//相移
        //Call Initiall
        HallIntial(&Obj->HallCalc);  
        //Hall Study initial
        Obj->HallStudy.CurrentValue = 300;
        
        /***************	motor 2 parameters  ******************/
        Obj1->HallCalc.Pwm_freq = PWM_FREQUENCY2;
        Obj1->HallCalc.Pols = MOTOR_POPAIRS2;
        //state parameter
        Obj1->HallCalc.HallFixLineSeq[0] = 3;
        Obj1->HallCalc.HallFixLineSeq[1] = 1;
        Obj1->HallCalc.HallFixLineSeq[2] = 5;
        Obj1->HallCalc.HallFixLineSeq[3] = 4;
        Obj1->HallCalc.HallFixLineSeq[4] = 6;
        Obj1->HallCalc.HallFixLineSeq[5] = 2;
        Obj1->HallCalc.HallFixAngle[0] = 24334;//相移
        //Call Initiall
        HallIntial(&Obj1->HallCalc);  
        //Hall Study initial
        Obj1->HallStudy.CurrentValue = 300;
    #endif 
}

void FocLoopReIni(Motor_Obj *pObj)
{
    pObj->SpeedObj.target_calc = 0;
    pObj->SpeedObj.UsDifSum = 0;
    pObj->CurLoop.IdFilterTmp = 0;
    pObj->CurLoop.IqFilterTmp = 0;
    pObj->CurLoop.IdSumTemp = 0;
    pObj->CurLoop.IqSumTemp = 0;
}
void FocParamReIni(Motor_Obj *pObj)
{			
    FocLoopReIni(pObj);
}
// ========================================================================
// 函数名称：MotorMain_Circle
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：主函数循环
// ========================================================================
void ForMotorMain(SystemInterface_Obj *InfObj,SystemInterface_Obj *InfObj1)
{
		#ifdef UART_SEND	
	Uart_Deal(&motor_I[0],&motor_I[1]);
		#endif	
		#ifdef SPI_SEND	
	Spi_Deal(&motor_I[0],&motor_I[1]);
		#endif		
		#ifdef Offset_SEND	
	OffsetValue_Send(&motor_I[0]);
		#endif		
}
// ========================================================================
// 函数名称：Spi_Deal
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：SPI发送数据处理
// ========================================================================
extern int16_t TestCount;
void Spi_Deal(Motor_Obj *Obj,Motor_Obj *Obj1)
{
	DAC_update(1,((Obj->DataObj.Ia_Q15)+2048));	
	DAC_update(2,((Obj->SpeedObj.Speed_Target>>4)));	
	DAC_update(3,((Obj->DataObj.Ic_Q15>>3)+2048));
}
// ========================================================================
// 函数名称：Uart_Deal
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：串口发送数据处理
// ========================================================================
void Uart_Deal(Motor_Obj *Obj,Motor_Obj *Obj1)
{
	  SendFixView_c(Obj,Obj1);
	if(mRecvFrame.State == 2) // 数据接收完成
	{
		mRecvFrame.index = SubCharFill(mRecvFrame.Buf,mRecvFrame.index);
		if(mRecvFrame.index > 0) 
        {
			ExploreParamRoot_c(Obj,Obj1,mRecvFrame.Buf,mRecvFrame.index); //  数据分类并回复数据					
		}
		mRecvFrame.State = 0; // 回复完成，继续接收数据
	}
	delay_100us(1);	
}
// ========================================================================
// 函数名称：Uart_Isr
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：串口中断函数
// ========================================================================
void Uart_Isr(uint8_t data_in)
{
	switch(mRecvFrame.State)
	{
		case 0:
            if(data_in == 0x7E) 
            {
                mRecvFrame.State = 1;
                mRecvFrame.index = 0;
            }
            break;
		case 1:
            if(data_in == 0x7E) 
            {
                if(mRecvFrame.index > 0) 
                {
                        mRecvFrame.State = 2;
                }
            }
            else if(mRecvFrame.index < ONE_FRAME_LEN) 
            {
                mRecvFrame.Buf[mRecvFrame.index++] = data_in;
            }
            break;
		case 2:
            break;            
		default:
            mRecvFrame.State = 0;
	}
}
// ========================================================================
// 函数名称：Udc_Deal
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：Udc_Deal计算
// ========================================================================
void Udc_Deal(uint8_t num,Motor_Obj *Obj,SystemInterface_Obj *InfObj)
{
	Obj->DataObj.Udc_Q15 = (*InfObj->UdcSample_I << 3);
	Obj->DataObj.Udc_Real = ((uint32_t)Obj->DataObj.Udc_Q15 * Obj->MotorBaseObj.VolatageBase) >> 15;
	if (num == 0)
	{
		Obj->CurLoop.Us2 = (LIMIT_DUTY1 * LIMIT_DUTY1);
	}
	else if (num == 1)
	{
		Obj->CurLoop.Us2 = (LIMIT_DUTY2 * LIMIT_DUTY2);
	}
}
// ========================================================================
// 函数名称：MotorIsr_200us
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：电机层200us中断函数
// ========================================================================
void MotorIsr_200us(uint8_t num,SystemInterface_Obj *InfObj)
{
	motor_I[num].DelayCnt++;
	if (motor_I[num].DelayCnt == 10)
	{
		motor_I[num].DelayCnt = 0;
    #ifdef HALL_FOR_ANGLE_T
		Speed_air_2ms(num,&motor_I[num]);
		#endif				
	}
	else
	{
		ElectUpDected_c(&motor_I[num]);		
		StartStop_Isr(&motor_I[num],num);
		ErrDeal_Isr(&motor_I[num]);
		IzeroReCalc_c(&motor_I[num],InfObj);			
		SpeedGiven();
		
        #ifdef HALL_FOR_ANGLE_T
		motor_I[num].DataObj.MotorSpeed = HallSpeedGet(&motor_I[num].HallCalc);
        #endif											
	}
}
// ========================================================================
// 函数名称：uint8_t motor_num
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：电机层PWM中断函数
// ========================================================================
void MotorPwm_Isr_I(uint8_t num,SystemInterface_Obj *InfObj)
{
	if (motor_I[num].Flag.Bits.MotorStarted == 1)
	{			 
		CurrentQ15Calcu_c(&motor_I[num],InfObj);				
		IdIqFbCalcu_c(&motor_I[num]);				
    BrushlessPiCtrl(num,&motor_I[num]);
		#ifdef HALL_FOR_ANGLE_T
		motor_I[num].BrushlessObj.EleAng_Q15 = HallAngleCalc(&motor_I[num].HallCalc,GetIO_GetHall(num));
		#endif 					
		SvpwmNew_Isr_c(num,&motor_I[num],motor_I[num].CurLoop.UdOut_Q15,motor_I[num].CurLoop.UqOut_Q15);
    motor_I[num].ClearOnce = 0;
	}
	else
	{
        if (motor_I[num].ClearOnce < 3)
        {														
            motor_I[num].ClearOnce++;
					  SvpwmInit_c(&motor_I[num],InfObj->Half_duty_I);					
            DataDealInit_c(&motor_I[num]);					
            CurLoopInit_c(&motor_I[num]);										
            BrushlessInit(&motor_I[num]);
            SpeedCtrlInit(num,&motor_I[num]);           
										
            #ifdef HALL_FOR_ANGLE
            HallIntial(&motor_I[num].HallCalc);
            #endif
        }
	}
	PwmDutySet_I(num,motor_I[num].Svpwm.DutyA,motor_I[num].Svpwm.DutyB,motor_I[num].Svpwm.DutyC);
}
// ========================================================================
// 函数名称：BrakeErrIsr_I
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：刹车故障中断
// ========================================================================
void BrakeErrIsr_I(uint8_t num)
{
    motor_I[num].Flag.Bits.HwTotalErr = 1;
    motor_I[num].Flag.Bits.HwOverCurrent = 1;
}


