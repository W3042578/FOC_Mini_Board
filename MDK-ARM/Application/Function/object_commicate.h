#ifndef __OBJECT_COMMICATE_H
#define __OBJECT_COMMICATE_H

#include "stm32f1xx_hal.h"

//索引		子索引		数据byte位数		通迅变量变量地址		变量最大值		变量最小值		调用函数指针

//定义结构体
typedef  struct
{
	uint16_t	Index;
	uint8_t		Sub_Index;
	uint8_t		Data_Type;
	void* 		Commicate_Pointor;
	uint16_t	Max;
	uint16_t	Min;
	uint16_t	(*Callback) (uint8_t Use_Sub_Index);
}commicate_code;


//通讯变量数目
extern uint16_t Commicate_Data_All_Number;
extern const commicate_code Commicate_Code[];


uint16_t Get_Communicte_Data_Number(void);
void Commicate_Data_Init(void);

#endif
