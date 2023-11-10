#ifndef __MODBUS_H
#define __MODBUS_H

#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "main.h"


#define Slave_ID 0x01    			//定义从机ID

//Modbus协议： ID号--ID   功能码--Function_Code  起始地址——Start_Address   寄存器数——Register_Number  字节数——Byte_Number  读取一个内容——Read_Data   
//获取的校验码——CRC_RX_Data		自身计算的校验码——CRC_Self_Data		写入一个内容——Write_Data		写入十个内容——Write_TEN_Data[10]
//数据帧包含字节长度——Modbus_Length	
union Modbus_CRC
{
	uint16_t ALL;
	struct
	{
		uint8_t  low;
		uint8_t  high;    //高位在下
	}nchar;
};
//定义8位与16位数组共用数据的接受区
union Modbus_Transfer_Data
{
	uint16_t ALL;
	struct
	{
		uint8_t Transfer_Data_8bit_high;
		uint8_t	Transfer_Data_8bit_low;
	}byte;
};

typedef struct
{
	uint8_t		ID;																							//modbus通讯ID
	uint8_t		Function_Code;																	//modbus功能码
	uint16_t	Start_Address;																	//开始地址
	uint16_t	Register_Number;																//寄存器个数
	uint8_t		Byte_Number;																		//字节个数
	union 	Modbus_Transfer_Data 	Read_Data_Array[10];				//读数据数组
	union 	Modbus_Transfer_Data 	Write_Data_Array[10];				//写数据数组
	uint8_t 	Read_To_Send[20];																//读取用于发送的数据
	union 	Modbus_CRC  	CRC_RX_Data;												//接受到的校验码
	union 	Modbus_CRC  	CRC_Self_Data;											//以接受数据计算得到的校验码
	union 	Modbus_CRC		CRC_TX_Data;												//准备发送的校验码
	uint8_t		Error;																					//modbus错误标志
	uint16_t	Write_Data;																			//写入一个数据用
	uint8_t		Const_Columns_Number;														//获取当前操作数据列数
	uint8_t		Test;
}MODBUS;

//Modbus通讯
extern uint8_t Modbus_Length_In;
extern uint16_t Modbus_Length_Out;
extern MODBUS Modbus;
extern uint8_t Modbus_Buffer[30];

uint8_t Modbus_Process(uint8_t *data_in,uint8_t length_in,uint8_t *data_out,uint16_t* length_out);
uint16_t Modbus_CRC_Data(uint8_t *data,uint16_t length);



#endif
