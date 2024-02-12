
#ifndef __PARAMETER_H
#define __PARAMETER_H


#include "stm32f1xx_hal.h"
 

#define MT6815
#define Speed_Transfer	6.5536			//速度转换比例  1rpm -> 1 dec_diff/1ms = 6.5536


typedef  struct
{
	uint8_t 	Work_Model;		//1:校准  2:占空比  3:电压开环  4:电流环 5:速度环 6:位置环 7:速度环无感 
	uint8_t 	PWM_Enable;		//0:PWM关闭使能		1:PWM开始使能				
	uint8_t 	Energency_Stop;		//0：不使用急停 1:触发急停
	uint8_t 	Work_Direction;		//0:正转		1:反转

	uint8_t		Open_Loop_Voltage;	//开环电压
	uint8_t 	Max_Voltage;		//最大电压限制
	uint8_t		Angle_Initial_Voltage;	//编码器线性校正和初始位置置零Ud电压
	uint8_t   	Number_Angle_Offest;	//初始角校正累加次数= 2的n次方
	uint8_t		Clear_Position;		//重置当前位置为0

	uint8_t		Duty_Model_A;		//占空比模式三相输入值
	uint8_t		Duty_Model_B;
	uint8_t		Duty_Model_C;
	
}_Control_Word;



union _Error_Message
{
		uint16_t	All;
		struct
		{
			uint8_t		ADC_Error:1;			//电流采样错误
			uint8_t		Bus_Voltage:1;			//总线电压错误	
			uint8_t		Encoder_Status:1;		//编码器状态错误
			uint8_t		Modbus_Status:1;		//Modbus通讯状态错误
			uint8_t		IIC_Status:1;			//IIC通讯状态错误
			uint8_t		Control_Loop_Error:1;		//控制环路错误
			uint8_t		Phase_Lose:1;			//相丢失错误  电机没有按照指令正常运动
		}bits;
};

union _Work_Status
{
		uint16_t All;
		struct
		{
			uint8_t 	Enable_Status:1;		//PWM使能状态
			uint8_t 	Angle_Offest:1;			//编码器和电流获取修正值状态
			uint8_t 	Duty_Model_Status:1;  	//占空比模式状态
			uint8_t		Offest_Encoder:1;		//编码器零位校准
			uint8_t		Direction_Encoder:1;	//编码器方向判断
			uint8_t		Linear_Compensation_Encoder:1;	//编码器线性补偿
			uint8_t		Offest_Current:1;		//电流修正
			uint8_t		First_Start:1;			//初次启动
			uint8_t		Ready_On:1;				//控制器准备就绪
			uint8_t		Control_Loop_Error:1;	//控制环路错误
			uint8_t		Phase_Lose:1;			//相丢失错误  电机没有按照指令正常运动
			uint8_t		Encoder_Init:1;			//编码器启动计数位置初次操作
			uint8_t		Interrupt_1MS_Init:1;	//1ms首次进入标志位
		}bits;	
};

//控制状态变量
extern	_Control_Word Control_Word; 
extern	union _Error_Message Error_Message;
extern  union _Work_Status Work_Status;
extern int16_t Number_Encoder_Direction;

//变量用于编码器通讯角度获取 测试验证用
extern uint16_t Transfer1[3];


//初始化
void Hardware_Init(void);//硬件参数初始化
void Error_Message_Init(union _Error_Message *Message);//控制字初始化
void Work_Status_Init(union _Work_Status *Status);//错误状态初始化
void Parameter_Init(void);		//上层参数初始化

#endif
