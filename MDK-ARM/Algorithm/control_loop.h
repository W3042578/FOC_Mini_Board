#ifndef __CONTROL_LOOP_H
#define __CONTROL_LOOP_H

#include "stm32f1xx_hal.h"

typedef  struct		//PID参数
{
	uint16_t 	Proportion;						  
	uint16_t 	Integral;								
	uint16_t 	Difference;

	int32_t		Error,Last_Error,Expect,Feedback;	
	int32_t		Proportion_Sum;
	int32_t		Integral_Sum;
	int32_t		Difference_Sum;
	int32_t		Output_Sum;
	
	uint32_t  	Proportion_Limit;
	uint32_t	Integral_Limit;
	uint32_t	Difference_Limit;
	uint32_t	Output_limit;		//输出限制
}_PID_Control;

typedef struct 		//前馈参数
{
	int32_t		Forward_Feedback;	//前馈量
	uint16_t 	Forward_Data[4];	//前馈模型参数
	uint16_t	Forward_Porportion;	//前馈系数
}_Forward;

typedef  struct		//环路控制参数
{
	int32_t 		Input_Target;		//输入目标 
	int32_t 		Output_Result;		//输出结果
	int32_t			Back;				//回馈
	_PID_Control	Loop_PID;			//控制环PID参数
	_Forward		Forward;			//前馈模型参数
	union
	{
		uint32_t ALL;					//环路控制字
		struct 
		{
			uint8_t Loop_Model:5;		//环路工作模式
			uint8_t Loop_Control:4;		//环路控制模式
			uint8_t Loop_Statue:4;		//环路状态
		}bits;	
	}Word;

}_Control_Loop;

enum Loop_Control_Word		//环路控制字枚举
{
	NO_USE 			= 0,	//PID结果直接输出
	PID_FILTER		= 1,	//对PID结果滤波输出
	FORWARD_CONTROL = 2,	//使用前馈控制
	FORWARD_FILTER	= 3,	//PID结果结合前馈结果滤波
};
	
void Parallel_PID(_PID_Control *pid_control);	//抗饱和并联PID
void Control_Loop_Init(_Control_Loop *loop);	//PID控制环具体参数初始化
void Current_Loop_Control(_Control_Loop *loop);	//三环控制
void Speed_Loop_Control(_Control_Loop *loop);
void Position_Loop_Control(_Control_Loop *loop);

//全局变量
extern  _Control_Loop	Current_Q_Loop;
extern  _Control_Loop	Current_D_Loop;
extern	_Control_Loop 	Speed_Loop;
extern	_Control_Loop 	Position_Loop;
extern	_PID_Control 	Current_Q_PID;
extern	_PID_Control 	Current_D_PID;
extern	_PID_Control 	Speed_PI;
extern	_PID_Control 	Position_P;
extern	uint16_t		Loop_Count;	

#endif

