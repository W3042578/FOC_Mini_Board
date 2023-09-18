#ifndef __CONTROL_LOOP_H
#define __CONTROL_LOOP_H

#include "stm32f1xx_hal.h"
#include "stdint.h"


typedef struct _FOC_PID
{
	uint16_t Kp,Ki,Kd;																		//PID参数
	int32_t Expect,Feed_Back,Error,Last_Error;						//期望、回馈、误差、上次误差
	int32_t Integral,Different,Output;										//积分、微分、输出
	
	int32_t Error_Low_Limit,Error_Upper_Limit;						//误差上下限
	int32_t Integral_Low_Limit,Integral_Upper_Limit;			//积分上下限
	int32_t Different_Low_Limit,Different_Upper_Limit;		//微分上下限
	int32_t Output_Low_Limit,Output_Upper_Limit;					//输出上下限
	
}FOC_PID;


void Current_PID_Init(FOC_PID *foc_pid);
void Speed_PID_Init(FOC_PID *foc_pid);
void Position_PID_Init(FOC_PID *foc_pid);

void PID_Control(FOC_PID* foc_pid);


extern FOC_PID	FOC_PID_Current_Iq;									//定义全局电流环Iq PID结构体
extern FOC_PID	FOC_PID_Current_Id;									//定义全局电流环Id PID结构体
extern FOC_PID	FOC_PID_Speed;											//定义全局速度环PID结构体
extern FOC_PID	FOC_PID_Position;										//定义全局位置环PID结构体
extern uint16_t Loop_Time_Count;									//控制环路时间计算，1Dec = 62.5us

#endif

