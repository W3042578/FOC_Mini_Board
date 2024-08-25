
#include "parameter.h"
#include "foc.h"
#include "encoder.h"
#include "object_commicate.h"
#include "control_loop.h"
//宏定义

//全局变量
_Control_Data Control_Data;				//控制变量定义
_Control_Status	Control_Status;			//控制状态定义

//硬件参数初始化
void Hardware_Init(void)
{
	//MCU配置
	Motor1.Ts_Count = TS_COUNT;//16k 
	
	//电机
	Motor1.Polar = 11;
	Motor1.Udc = 12;			//母线工作电压为 12V

	//驱动板
	Driver1.Dead_Time = DEAD_TIME;		//ns
	//0.1A电流对应adc值
	Driver1.Scale_1_10 = (2 << (ADC_BIT - 1)) * ADC_RESISTANCE * ADC_SCALE / (10000 * ADC_MAX_VOLATGE);
}

//控制数据初始化
void Control_Data_Init(_Control_Data *Data)
{
	//工作模式
	Data->Control_Word.All = 0;
	Data->Control_Word.bits.Work_Model = 1;
	Data->Control_Word.bits.Sub_Work_Model = 6;
	
	//编码器
	Data->Control_Word.bits.Encoder_Type = KTH7812;	

	//电压开环
	Data->Open_Loop_Voltage = 1;		//开环电压
	Data->Max_Voltage = 12;				//最大母线电压限制

	//编码器校正
	Data->Encoder_Offest.Angle_Initial_Voltage = 1;	//编码器线性校正Ud电压
	Data->Encoder_Offest.Number_Angle_Offest = 3;	//初始角校正累加次数= 2的n次方
	
	//占空比模式
	Data->Duty_Data.Phase_A = 50;		//占空比模式三相输入值
	Data->Duty_Data.Phase_B = 50;
	Data->Duty_Data.Phase_C = 50;

	//环路数据
	Data->Encoder_Offest.Virtual_Increment = VIRTUAL_INCREMENT;	//编码器校正虚拟角度增量
	Current_Q_PID.Proportion = 2;	//PID参数	
	Current_Q_PID.Integral = 1;
	Current_D_PID.Proportion = 2;
	Current_D_PID.Integral = 1;
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
	Hardware_Init();						//硬件参数初始化
	Commicate_Data_Init();					//modbus通讯数据初始化
	Control_Data_Init(&Control_Data);		//控制数据初始化
	Control_Status_Init(&Control_Status);	//控制状态初始化
	Loop_Init();							//控制环路初始化
	Encoder_Init(&Encoder1);				//编码器数据初始化
}

//状态位操作
void 	_SET(uint8_t * data,uint8_t bit)
{
	*data = (*data) | bit;
}
void 	_CLEAN(uint8_t * data,uint8_t bit)
{
	*data = (*data) & (~bit);
}
void	_NEGA(uint8_t * data,uint8_t bit)
{
	*data = *data ^ bit;
}
uint8_t	_TEST(uint8_t * data,uint8_t bit)
{
	if(((*data) & bit) != 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}	
