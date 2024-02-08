#ifndef __CONTROL_LOOP_H
#define __CONTROL_LOOP_H

#include "stm32f1xx_hal.h"


typedef  struct
{
	uint16_t 	Proportion;						  
	uint16_t 	Integral;								
	uint16_t 	Difference;
	uint16_t	Antiback;
	
	int32_t		Error,Last_Error,Expect,Feedback,Feedforward;	
	int32_t		Proportion_Sum;
	int32_t		Integral_Sum;
	int32_t		Difference_Sum;
	int32_t		Output_Sum;
	
	uint32_t  	Proportion_Limit;			//输出限制
	uint32_t	Integral_Limit;
	uint32_t	Difference_Limit;
	uint32_t	Output_limit;
}_PID_Control;

typedef  struct
{
	int32_t 	Target_Q_Current;	//目标 
	int32_t 	Target_D_Current;
	int32_t 	Target_Speed;								
	int32_t 	Target_Position;
	uint8_t		Loop_Count;		//环路计数，内环运行频率必须大于外环，否则环路震荡

	uint16_t	Current_Q_Proportion;	//q轴电流PID
	uint16_t	Current_Q_Integral;
	uint16_t	Current_Q_Difference;
	
	uint16_t	Current_D_Proportion;	//d轴电流PID
	uint16_t	Current_D_Integral;
	uint16_t	Current_D_Difference;

	uint16_t	Speed_Proportion;	//速度PI
	uint16_t	Speed_Integral;
		
	uint16_t	Position_Proportion;	//位置PI
	uint16_t	Position_Integral;		
	uint16_t 	Position_Feedforward;	
	
	int32_t  	Current_Q_Output_Limit;	//输出限制
	int32_t  	Current_D_Output_Limit;	
	int32_t		Speed_Output_Limit;
	int32_t		Position_Output_Limit;
}_Control_Loop;


extern	_Control_Loop	Control_Loop;
extern	_PID_Control 	Current_Q_PID;
extern	_PID_Control 	Current_D_PID;
extern	_PID_Control 	Speed_PI;
extern	_PID_Control 	Position_PI;

//PID计算
void PID_Control_Deal(_PID_Control * pid_control);
//PID结构体参数初始化
void PID_Control_Init(_PID_Control *pid);
//PID控制环具体参数初始化
void Control_Loop_Init(_Control_Loop *loop);
//1ms中断更新PID结构体中数据
void PID_Control_Update(void);

#endif

