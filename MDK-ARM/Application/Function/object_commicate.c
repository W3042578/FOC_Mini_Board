#include "main.h"
#include "object_commicate.h"
#include "modbus.h"
#include "parameter.h"
#include "control_loop.h"

uint16_t Commicate_Data_All_Number;	//通许数据量


//通许变量数组
//索引		子索引		字节数byte		变量地址		最大值		最小值		函数指针
const commicate_code Commicate_Code[] =
{
	{0x0000,	0x00,	1,	&Modbus.Test,																	255,	0,	NULL},
	{0x0001,	0x00,	1,	&Control_Word.Work_Model,			255,	0,	NULL},
	{0x0002,	0x00,	1,	&Control_Word.Number_Angle_Offest,		255,	0,	NULL},//初始角校正累加数
	{0x0003,	0x00,	1,	&Control_Word.PWM_Enable,			255,	0,	NULL},//0不触发 1:触发
	{0x0004,	0x00,	1,	&Control_Word.Energency_Stop,			255,	0,	NULL},//0不触发 1:触发
	{0x0005,	0x00,	1,	&Control_Word.Open_Loop_Voltage,		255,	0,	NULL},
	{0x0006,	0x00,	1,	&Control_Word.Work_Direction,			255,	0,	NULL},//0:正		1:反
	{0x0007,	0x00,	1,	&Control_Word.Max_Voltage,			255,	0,	NULL},
	{0x0008,	0x00,	2,	&Control_Loop.Target_Current,			65535,	0,	NULL},
	{0x0009,	0x00,	2,	&Transfer1[0],					65535,	0,	NULL},
	{0x000A,	0x00,	2,	&Control_Loop.Target_Position,			65535,	0,	NULL},
	{0x000B,	0x00,	2,	&Control_Loop.Current_Q_Proportion,		65535,	0,	NULL},
	{0x000C,	0x00,	2,	&Control_Loop.Current_Q_Integral,		65535,	0,	NULL},
	{0x000D,	0x00,	2,	&Control_Loop.Current_Q_Difference,		65535,	0,	NULL},	
	{0x000E,	0x00,	2,	&Control_Loop.Current_D_Proportion,		65535,	0,	NULL},
	{0x000F,	0x00,	2,	&Control_Loop.Current_D_Integral,		65535,	0,	NULL},
	{0x0010,	0x00,	2,	&Control_Loop.Current_D_Difference,		65535,	0,	NULL},
	{0x0011,	0x00,	2,	&Control_Loop.Speed_Proportion,			65535,	0,	NULL},
	{0x0012,	0x00,	2,	&Control_Loop.Speed_Integral,			65535,	0,	NULL},
	{0x0013,	0x00,	2,	&Control_Loop.Position_Proportion,		65535,	0,	NULL},
	{0x0014,	0x00,	2,	&Control_Loop.Position_Integral,		65535,	0,	NULL},
	{0x0015,	0x00,	2,	&Control_Loop.Position_Output_Limit,		65535,	0,	NULL},
	{0x0016,	0x00,	2,	&Control_Loop.Position_Feedforward,		65535,	0,	NULL},
	{0x0017,	0x00,	2,	&Error_Message,					65535,	0,	NULL},
	{0x0018,	0x00,	2,	&Control_Loop.Current_Q_Output_Limit,		65535,	0,	NULL},
	{0x0019,	0x00,	2,	&Control_Loop.Current_D_Output_Limit,		65535,	0,	NULL},
	{0x001A,	0x00,	2,	&Control_Loop.Speed_Output_Limit,		65535,	0,	NULL},
	{0x001B,	0x00,	2,	&Control_Loop.Speed_Output_Limit,		65535,	0,	NULL},
	{0x001C,	0x00,	2,	&Control_Loop.Speed_Output_Limit,		65535,	0,	NULL},
	{0x001D,	0x00,	2,	&Control_Loop.Speed_Output_Limit,		65535,	0,	NULL},
	{0x001E,	0x00,	2,	&Transfer1[2],					65535,	0,	NULL}
};

//获取通许数据量
uint16_t Get_Communicte_Data_Number(void)
	
{
	uint16_t data_all,data_single;
	uint16_t Output;
	
	data_all = sizeof(Commicate_Code);
	data_single = sizeof(Commicate_Code[0]);
	Output = data_all/data_single;
	
	return Output;
}
//modbus通讯数据初始化,获取通讯变量个数并初始化控制变量
void Commicate_Data_Init(void)
{
	Commicate_Data_All_Number = Get_Communicte_Data_Number();
	Error_Message.All = 0;
}


