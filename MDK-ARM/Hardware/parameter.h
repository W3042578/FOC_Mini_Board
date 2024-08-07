
#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "stm32f1xx_hal.h"
 
//置位定义
#define	 BIT0  0x01
#define	 BIT1  0x02
#define	 BIT2  0x04	
#define	 BIT3  0x08	
#define	 BIT4  0x10	
#define	 BIT5  0x20	
#define	 BIT6  0x40	
#define	 BIT7  0x80	

typedef union
{
	uint32_t All;
	struct 
	{
		uint8_t 	Work_Model:4;			//工作模式 
		uint8_t 	Work_Model_Buffer:4;	//过去工作模式 
		uint8_t		Sub_Work_Model:4;		//工作子模式
		uint8_t		Sub_Work_Model_Buffer:4;//过去工作子模式
		uint8_t 	PWM_Enable:1;			//0:PWM关闭使能		1:PWM开始使能			
		uint8_t 	Energency_Stop:1;		//0:无急停 1:触发急停
		uint8_t 	Work_Direction:1;		//0:正转		1:反转
		uint8_t		Clear_Position:1;		//重置当前位置为0
		uint8_t		MTPA:1;					//电流环MTPA模式控制
		uint8_t		Current_Forward:1;		//电流环前馈解耦控制
		uint8_t		Encoder_Type:4;			//编码器类型
	}bits;
}_Control_Word;

typedef  struct
{
	_Control_Word Control_Word;			//控制字
	uint8_t		Open_Loop_Voltage;		//开环电压
	uint8_t 	Max_Voltage;			//最大母线电压

	struct
	{
		int32_t		Position_Pulse;			//累计脉冲值
		int32_t		Position_Pulse_Buffer;	//累计脉冲值缓冲
		int32_t		Position_Loop_Increment;//位置环周期位置增量
	}Pulse_Increment;			//脉冲增量
	
	struct
	{
		uint8_t		Angle_Initial_Voltage;	//位置对齐校正Ud电压
		uint8_t   	Number_Angle_Offest;	//位置数据要求累加次数= 2的n次方
		uint16_t	Number_Offest_Count;	//位置数据获取计数
		uint32_t	Offest_Integral;		//位置数据累加存储值
		uint16_t	Virtual_Angle;			//虚拟电角度
		int32_t		Differ_Check;			//检查角度缓冲数组
		uint16_t	Offest_Table_Count;		//线性补偿计数
		uint16_t	Offest_Wait;			//位置数据获取间隔时间 单位：位置环周期
		uint8_t		Offest_Model;			//校正内置模式 1：零位对齐 2：正向 3：反向
		uint8_t		Error_Time;				//检查角度错误次数
	}Encoder_Offest;			//编码器修正参数
	struct 
	{
		uint8_t		Phase_A;			
		uint8_t		Phase_B;
		uint8_t		Phase_C;
	}Duty_Data;					//占空比模式三相输入值
	
	struct						
	{
		uint8_t Position_Source;
		uint8_t Speed_Source;
		uint8_t Current_Source;
		uint8_t Voltage_Source;
	}Control_Source;			//环路控制输入来源
	
}_Control_Data;

typedef union
{
		uint32_t All;
		struct
		{
			uint8_t 	Enable_Status:1;		//PWM使能状态
			uint8_t 	Angle_Offest:1;			//编码器和电流获取修正值状态
			uint8_t 	Duty_Model_Status:1;  	//占空比模式状态
			uint8_t		Offest_Encoder:1;		//编码器零位校准
			uint8_t		Offest_Current:1;		//电流修正
			uint8_t		Direction_Encoder:1;	//编码器方向判断
			uint8_t		First_Start:1;			//初次启动
			uint8_t		Ready_On:1;				//控制器准备就绪
			uint8_t		Encoder_Init:1;			//编码器启动计数位置初次操作
			uint8_t		Interrupt_1MS_Init:1;	//1ms首次进入标志位
		}bits;	
}_Work_Status;

typedef union
{
		uint32_t	All;
		struct
		{
			uint8_t		ADC_Error:1;			//电流采样错误
			uint8_t		Bus_Voltage:1;			//总线电压错误	
			uint8_t		Encoder_Status:1;		//编码器状态错误
			uint8_t		Encoder_Offset:1;		//编码器校正错误
			uint8_t		Modbus_Status:1;		//Modbus通讯状态错误
			uint8_t		IIC_Status:1;			//IIC通讯状态错误
			uint8_t		Control_Loop_Error:1;	//控制环路错误
			uint8_t		Phase_Lose:1;			//相丢失错误  电机没有按照指令正常运动
			uint8_t		PWM_Enable:1;			//使能关闭错误
		}bits;
}_Error_status;

typedef  struct
{
	_Work_Status		Work_Status;			//工作状态
	_Error_status		Error_status;			//错误状态信息

}_Control_Status;

//控制状态变量
extern	_Control_Data	Control_Data;			//控制数据
extern	_Control_Status	Control_Status;			//控制状态

//变量用于编码器通讯角度获取 测试验证用
extern uint16_t Transfer1[3];

//初始化
void Parameter_Init(void);							//参数初始化
void Hardware_Init(void);							//硬件参数初始化
void Control_Data_Init(_Control_Data *Data);		//控制数据初始化
void Control_Status_Init(_Control_Status *Status);	//控制状态初始化

//数据更新
void Control_Data_Update(_Control_Data *Word);		//控制数据更新
void Control_Status_Update(_Control_Status *Status);//控制状态更新

//标志位处理
void _SET(uint8_t * data,uint8_t bit);
void _CLEAN(uint8_t * data,uint8_t bit);
uint8_t _TEST(uint8_t * data,uint8_t bit);

#endif
