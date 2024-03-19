
#include "parameter.h"
#include "foc.h"
#include "encoder.h"
#include "object_commicate.h"
#include "control_loop.h"

union _Error_Message Error_Message;		//错误信息指示位
union _Work_Status Work_Status;			//工作状态指示位
union _Control_Word Control_Word; 		//通许变量位

_Control_Data Control_Data;				//控制变量定义

int16_t Number_Encoder_Direction;		//待定

#define Motor1_Dead_Time	2		//电机死区时间 单位us
//硬件参数初始化
void Hardware_Init(void)
{
	//MCU配置
	//设置工作时间Ts = 2*(2+1)*(1499+1)/72M = 125us  中心对齐模式x2
	Motor1.Ts = 2249;//16k频率下定时器计数值  满额2249  
	
	//电机
	Motor1.Polar = 11;
	Motor1.Udc = 12;			//母线电压为 12V
	//此处Td为16位数据，最大值65536，Motor1_Dead_Time不能超过3.5us,否则数据会溢出导致计算错误
	Motor1.Td = 8.192 * Motor1_Dead_Time *（Motor1.Ts + 1）>> 9;
		
	//编码器
	Encoder1.Type = 1; 			//SPI通讯编码器
	Encoder1.Single_Bit = 14;	//14位单圈
	Encoder1.Multi_Bit = 0;		//多圈位为0
	
	
}



//控制字初始化
void Control_Word_Init(union _Control_Word *Word)
{
	Word->All = 0;
}

//控制字初始化
void Control_Data_Init(_Control_Data *Data)
{
	Data->Open_Loop_Voltage = 0;		//开环电压置零
	Data->Angle_Initial_Voltage = 3;	//编码器线性校正Ud电压
	Data->Number_Angle_Offest = 5;		//初始角校正累加次数= 2的n次方
	Data->Max_Voltage = 12;				//最大母线电压限制
	Data->Duty_Model_A = 50;			//占空比模式三相输入值
	Data->Duty_Model_B = 50;
	Data->Duty_Model_C = 50;
}

//错误状态初始化
void Error_Message_Init(union _Error_Message *Message)
{
	Message->All = 0;
}	

//工作状态初始化
void Work_Status_Init(union _Work_Status *Status)
{
	Status->All = 0;
}

//上层参数初始化
void Parameter_Init(void)
{
	//硬件参数初始化
	Hardware_Init();
	
	//modbus通讯数据初始化
	Commicate_Data_Init();
	
	//控制字初始化
	Control_Word_Init(&Control_Word);
	//控制变量初始化
	Control_Data_Init(&Control_Data);
	
	//错误状态初始化
	Error_Message_Init(&Error_Message);
	
	//工作状态初始化
	Work_Status_Init(&Work_Status);

	//各控制环PID参数初始化
	PID_Control_Init(&Current_Q_PID);
	PID_Control_Init(&Current_D_PID);
	PID_Control_Init(&Speed_PI);
	PID_Control_Init(&Position_PI);

	//控制环参数初始化
	Control_Loop_Init(&Control_Loop);
}
