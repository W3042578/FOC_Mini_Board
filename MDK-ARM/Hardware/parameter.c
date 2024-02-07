#include "parameter.h"

#include "foc.h"
#include "encoder.h"
#include "object_commicate.h"
#include "control_loop.h"

union _Error_Message Error_Message;		//错误信息指示位
union _Work_Status Work_Status;				//工作状态指示位

_Control_Word Control_Word; 					//通许变量、控制变量定义
_Control_Loop	Control_Loop;						//控制环路变量

int16_t Number_Encoder_Direction;			//待定

//硬件配置
void Hardware_Init(void)
{
	//电机
	Motor1.Polar = 11;
	Motor1.Udc = 12;						//母线电压为 12V
		
	//编码器
	encoder1.Type = 1; 					//SPI通讯编码器
	encoder1.Single_Bit = 14;		//14位单圈
	encoder1.Multi_Bit = 0;			//多圈位为0
	
	//MCU配置
	//设置工作时间Ts = 2*(2+1)*(1499+1)/72M = 125us  中心对齐模式x2
	Motor1.Ts = 2249;//16k频率下定时器计数值  满额2249  
}



//控制字
void Control_Word_Init(_Control_Word *Data)
{
	Data->Work_Model = 0;							//1:校准  2:占空比  3:电压开环  4:电流环 5:速度环 6:位置环 7:速度环无感
	Data->PWM_Enable = 0;							//0:PWM关闭使能		1:PWM开始使能
	Data->Energency_Stop = 0;					//1:进入紧急停止
	Data->Work_Direction = 0;					//0:当前方向 1:当前反向
	Data->Open_Loop_Voltage = 0;			//开环电压
	Data->Max_Voltage = 16;						//最大电压限制
	Data->Angle_Initial_Voltage = 5;	//编码器线性校正和初始位置置零Ud电压
	Data->Number_Angle_Offest = 5;		//次数 = 2的n次方
	Data->Clear_Position = 0;					//重置当前位置为0
	
	Data->Duty_Model_A = 50;					//占空比模式三相初始值
	Data->Duty_Model_B = 50;
	Data->Duty_Model_C = 50;
}

//错误字
void Error_Message_Init(union _Error_Message *Message)
{
	Message->All = 0;
}	

//工作状态字
void Work_Status_Init(union _Work_Status *Status)
{
	Status->All = 0;
}

//参数初始化
void Parameter_Init(void)
{
	//硬件参数初始化
	Hardware_Init();
	
	//modbus通讯数据初始化
	Commicate_Data_Init();
	
	//控制环参数初始化
	Control_Loop_Init(&Control_Loop);
	
	//控制字初始化
	Control_Word_Init(&Control_Word);
	
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
