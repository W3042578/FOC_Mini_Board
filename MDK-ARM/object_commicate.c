#include "main.h"
#include "object_commicate.h"
#include "foc.h"
#include "usart_control.h"
#include "modbus.h"
#include "control_loop.h"

uint16_t Commicate_Data_All_Number;		//???????????????????????????A???????????

_Control_Word Control_Data; 					//???????????  ??????????
_Control_Loop	Control_Loop;
union _Error_Message Error_Message;

uint16_t test_one =  3;
uint16_t test_two	=  20;


//??????????
//Control_Data??????????main.c??,??????usart_control.h??
//????		??????		????byte????		?????????????		????????		?????????		???????????
const commicate_code Commicate_Code[] =
{
	{0x0000,	0x00,	1,	&Modbus.Test,																	255,	0,	NULL},
	{0x0001,	0x00,	1,	&Control_Data.Work_Model,											255,	0,	NULL},//?????? 0:???????   1:??????  2:????  3:?????
	{0x0002,	0x00,	1,	&Control_Data.Encoder_Offect_Process,					255,	0,	NULL},//0:??????????????? 1;?????????????
	{0x0003,	0x00,	1,	&Control_Data.Enable,													255,	0,	NULL},//0:???		1:????
	{0x0004,	0x00,	1,	&Control_Data.Energency_Stop,									255,	0,	NULL},//0?????????? 1:???????
	{0x0005,	0x00,	1,	&Control_Data.Open_Loop_Voltage,							255,	0,	NULL},
	{0x0006,	0x00,	1,	&Control_Data.Work_Direction,									255,	0,	NULL},//0:???		1:???
	{0x0007,	0x00,	1,	&Control_Data.Max_Voltage,										255,	0,	NULL},//??????????????15V
	{0x0008,	0x00,	2,	&Control_Loop.Target_Current,								65535,	0,	NULL},
	{0x0009,	0x00,	2,	&Control_Loop.Target_Speed,									65535,	0,	NULL},
	{0x000A,	0x00,	2,	&Control_Loop.Target_Position,							65535,	0,	NULL},
	{0x000B,	0x00,	1,	&Control_Loop.Current_PID_Proportion,					255,	0,	NULL},
	{0x000C,	0x00,	1,	&Control_Loop.Current_PID_Integral,						255,	0,	NULL},
	{0x000D,	0x00,	1,	&Control_Loop.Current_PID_Difference,					255,	0,	NULL},	
	{0x000E,	0x00,	1,	&Control_Loop.Speed_PID_Proportion,						255,	0,	NULL},
	{0x000F,	0x00,	1,	&Control_Loop.Speed_PID_Integral,							255,	0,	NULL},
	{0x0010,	0x00,	1,	&Control_Loop.Speed_PID_Difference,						255,	0,	NULL},
	{0x0011,	0x00,	1,	&Control_Loop.Positon_PID_Proportion,					255,	0,	NULL},
	{0x0012,	0x00,	1,	&Control_Loop.Positon_PID_Integral,						255,	0,	NULL},
	{0x0013,	0x00,	1,	&Control_Loop.Positon_PID_Difference,					255,	0,	NULL},
	{0x0014,	0x00,	2,	&Control_Loop.Current_Limit,								65535,	0,	NULL},
	{0x0015,	0x00,	2,	&Control_Loop.Speed_Limit,									65535,	0,	NULL},
	{0x0016,	0x00,	2,	&Control_Loop.Position_Limit,								65535,	0,	NULL},
	{0x0017,	0x00,	2,	&Error_Message,															65535,	0,	NULL},
	{0x0018,	0x00,	2,	&FOC_PID_Current_Iq.Error_Low_Limit,				65535,	0,	NULL},
	{0x0019,	0x00,	2,	&FOC_PID_Current_Iq.Error_Upper_Limit,			65535,	0,	NULL},
	{0x001A,	0x00,	2,	&FOC_PID_Current_Iq.Integral_Low_Limit,			65535,	0,	NULL},
	{0x001B,	0x00,	2,	&FOC_PID_Current_Iq.Integral_Upper_Limit,		65535,	0,	NULL},
	{0x001C,	0x00,	2,	&FOC_PID_Current_Iq.Different_Low_Limit,		65535,	0,	NULL},
	{0x001D,	0x00,	2,	&FOC_PID_Current_Iq.Different_Upper_Limit,	65535,	0,	NULL},
	{0x001E,	0x00,	2,	&FOC_PID_Current_Iq.Output_Low_Limit,				65535,	0,	NULL},
	{0x001F,	0x00,	2,	&FOC_PID_Current_Iq.Output_Upper_Limit,			65535,	0,	NULL},
	{0x0020,	0x00,	2,	&FOC_PID_Current_Iq.Output_Low_Limit,				65535,	0,	NULL},
	
	{0x0021,	0x00,	2,	&FOC_PID_Current_Id.Error_Low_Limit,				65535,	0,	NULL},
	{0x0022,	0x00,	2,	&FOC_PID_Current_Id.Error_Upper_Limit,			65535,	0,	NULL},
	{0x0023,	0x00,	2,	&FOC_PID_Current_Id.Integral_Low_Limit,			65535,	0,	NULL},
	{0x0024,	0x00,	2,	&FOC_PID_Current_Id.Integral_Upper_Limit,		65535,	0,	NULL},
	{0x0025,	0x00,	2,	&FOC_PID_Current_Id.Different_Low_Limit,		65535,	0,	NULL},
	{0x0026,	0x00,	2,	&FOC_PID_Current_Id.Different_Upper_Limit,	65535,	0,	NULL},
	{0x0027,	0x00,	2,	&FOC_PID_Current_Id.Output_Low_Limit,				65535,	0,	NULL},
	{0x0028,	0x00,	2,	&FOC_PID_Current_Id.Output_Upper_Limit,			65535,	0,	NULL},
	{0x0029,	0x00,	2,	&FOC_PID_Current_Id.Output_Low_Limit,				65535,	0,	NULL}
	
};

//??????????????? ????????? ????????????
//????  uint16_t Output ????????
uint16_t Get_Communicte_Data_Number(void)
	
{
	uint16_t data_all,data_single;
	uint16_t Output;
	
	data_all = sizeof(Commicate_Code);
	data_single = sizeof(Commicate_Code[0]);
	Output = data_all/data_single;
	
	return Output;
}
//??????????? 
void Commicate_Data_Init(void)
{
	Commicate_Data_All_Number = Get_Communicte_Data_Number();
	Error_Message.All = 0;
}


