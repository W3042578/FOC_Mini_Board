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
	int32_t		Integral_Data;
	int32_t		Difference_Sum;
	int32_t		Output_Sum;
	
	int32_t  	Proportion_Limit;
	int32_t		Integral_Limit;
	int32_t		Difference_Limit;
	int32_t		Output_limit;		//输出限制
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
	PID_FILTER		= 1,	//PID结果滤波输出
	FORWARD_CONTROL = 2,	//增加前馈
};

void Loop_Init(void);

void Open_Voltage_Loop(uint8_t *source,_Control_Loop *loop);	//电压开环
void Current_Loop(uint8_t *source,_Control_Loop *loop);	//三环控制
void Speed_Loop(uint8_t *source,_Control_Loop *loop);
void Position_Loop(uint8_t *source,_Control_Loop *loop);
	
uint8_t Current_Loop_Model(_Forward *forward);
uint8_t Speed_Loop_Data_Model(_Forward *forward);
uint8_t Position_Loop_Data_Model(_Forward *forward);

//全局变量
extern	_Control_Loop	Open_Voltage_Data;
extern  _Control_Loop	Current_Loop_Data;
extern	_Control_Loop 	Speed_Loop_Data;
extern	_Control_Loop 	Position_Loop_Data;
extern	_PID_Control 	Current_Q_PID;
extern	_PID_Control 	Current_D_PID;
extern	_PID_Control 	Speed_PI;
extern	_PID_Control 	Position_P;
extern	uint8_t			Loop_Count;	

#endif

