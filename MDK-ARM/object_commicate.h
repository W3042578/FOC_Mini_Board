#ifndef __OBJECT_COMMICATE_H
#define __OBJECT_COMMICATE_H

#include "main.h"

//索引		子索引		数据byte位数		通迅变量变量地址		变量最大值		变量最小值		调用函数指针

//定义结构体
typedef  struct
{
	uint16_t	Index;
	uint8_t	Sub_Index;
	uint8_t	Data_Type;
	void* 	Commicate_Pointor;
	uint16_t	Max;
	uint16_t	Min;
	uint16_t	(*Callback) (uint8_t Use_Sub_Index);
}commicate_code;

typedef  struct
{
		uint8_t 	Work_Model;							//0:电压开环   1:电流环  2:速度环  3:位置环  
		uint8_t		Encoder_Offect_Process;	//0:不进行编码器校准 1;进行编码器校准
		uint8_t 	Enable;									//0:停机		1:运行
		uint8_t		Enable_Buffer;					
		uint8_t 	Energency_Stop;					//0：不使用急停 1:触发急停
		uint8_t 	Work_Direction;					//0:正转		1:反转
		uint8_t		Open_Loop_Voltage;			//开环电压
		uint8_t 	Max_Voltage;						//允许设置最大电压为15V
}_Control_Word;

typedef  struct
{
		uint16_t 	Target_Current;							//目标电流  
		uint16_t 	Target_Speed;								
		uint16_t 	Target_Position;					
	
		uint8_t		Current_PID_Proportion;			//电流PID
		uint8_t		Current_PID_Integral;
		uint8_t		Current_PID_Difference;			
	
		uint8_t		Speed_PID_Proportion;				//速度PID
		uint8_t		Speed_PID_Integral;
		uint8_t		Speed_PID_Difference;				
	
		uint8_t		Positon_PID_Proportion;			//位置PID	
		uint8_t		Positon_PID_Integral;
		uint8_t		Positon_PID_Difference;			
		
		uint16_t  	Current_Limit;							//输出限制
		uint16_t	Speed_Limit;
		uint16_t	Position_Limit;
}_Control_Loop;

union _Error_Message
{
		uint16_t	All;
		struct
		{
			uint8_t		ADC_Error:1;						//电流采样错误
			uint8_t		Bus_Voltage:1;					//总线电压错误	
			uint8_t		Encoder_Status:1;				//编码器状态错误
			uint8_t		Modbus_Status:1;				//Modbus通讯状态错误
			uint8_t		IIC_Status:1;						//IIC通讯状态错误
			uint8_t		Control_Loop_Error:1;		//控制环路错误
			uint8_t		Phase_Lose:1;						//相丢失错误  电机没有按照指令正常运动
		}bits;
};

//通讯变量数目
extern uint16_t Commicate_Data_All_Number;
extern const commicate_code Commicate_Code[];

//通讯变量
extern	_Control_Word Control_Data; 
extern	_Control_Loop	Control_Loop;
extern	union _Error_Message Error_Message;



//通讯变量测试
extern uint16_t test_one;
extern uint16_t test_two;


uint16_t Get_Communicte_Data_Number(void);
void Commicate_Data_Init(void);

#endif
