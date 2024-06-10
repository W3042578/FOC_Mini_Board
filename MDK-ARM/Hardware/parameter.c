
#include "parameter.h"
#include "foc.h"
#include "encoder.h"
#include "object_commicate.h"
#include "control_loop.h"


_Control_Data Control_Data;				//控制变量定义
_Control_Status	Control_Status;			//控制状态定义

//硬件参数初始化
void Hardware_Init(void)
{
	//MCU配置
	//设置工作时间Ts_Count = 2*(2+1)*(1499+1)/72M = 125us  中心对齐模式x2  f103 72M主频
	Motor1.Ts_Count = 2249;//16k频率下定时器计数值  满额2249  
	
	//电机
	Motor1.Polar = 11;
	Motor1.Udc = 12;			//母线电压为 12V

	//驱动板
	Driver1.Dead_Time = 50;		//0.5us
	Driver1.ADC_Scale = 20;		//运放放大20
	Driver1.ADC_Resistance = 75;		//75毫欧

	//死区时间转换为比较值 后续考虑载波频率
	Motor1.Td_Count = (Driver1.Dead_Time * (Motor1.Ts_Count + 1)) / 6250;

}

//控制数据初始化
void Control_Data_Init(_Control_Data *Data)
{
	Data->Control_Word.All = 0;
	Data->Control_Word.bits.Encoder_Type = MT6813;	
	Data->Open_Loop_Voltage = 2;		//开环电压置零
	Data->Angle_Initial_Voltage = 5;	//编码器线性校正Ud电压
	Data->Number_Angle_Offest = 5;		//初始角校正累加次数= 2的n次方
	Data->Max_Voltage = 12;				//最大母线电压限制
	Data->Duty_Model_A = 50;			//占空比模式三相输入值
	Data->Duty_Model_B = 50;
	Data->Duty_Model_C = 50;
}

//控制状态初始化
void Control_Status_Init(_Control_Status *Status)
{
	Status->Work_Status.All = 0;
	Status->Error_status.All = 0;
}

//控制数据更新
void Control_Data_Update(_Control_Data *Word)
{

}

//控制状态更新
void Control_Status_Update(_Control_Status *Status)
{

}

//参数初始化
void Parameter_Init(void)
{
	Hardware_Init();		//硬件参数初始化

	Commicate_Data_Init();	//modbus通讯数据初始化
	
	Control_Data_Init(&Control_Data);		//控制数据初始化
	Control_Status_Init(&Control_Status);	//控制状态初始化

	PID_Control_Init(&Current_Q_PID);		//各控制环PID参数初始化
	PID_Control_Init(&Current_D_PID);
	PID_Control_Init(&Speed_PI);
	PID_Control_Init(&Position_P);

	Control_Loop_Init(&Current_Q_Loop);		//控制环参数初始化
	Control_Loop_Init(&Current_D_Loop);
	Control_Loop_Init(&Speed_Loop);
	Control_Loop_Init(&Position_Loop);

	Encoder_Init(&Encoder1);		//编码器数据初始化
}


//状态位操作
void _SET(uint8_t * data,uint8_t bit)
{
	*data = (*data) | bit;
}
void _CLEAN(uint8_t * data,uint8_t bit)
{
	*data = (*data) & (~bit);
}
uint8_t _TEST(uint8_t * data,uint8_t bit)
{
	if((*data) & bit != 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
