#ifndef __MODBUS_H
#define __MODBUS_H

#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "main.h"


#define Slave_ID 0x01    			//����ӻ�ID

//ModbusЭ�飺 ID��--ID   ������--Function_Code  ��ʼ��ַ����Start_Address   �Ĵ���������Register_Number  �ֽ�������Byte_Number  ��ȡһ�����ݡ���Read_Data   
//��ȡ��У���롪��CRC_RX_Data		��������У���롪��CRC_Self_Data		д��һ�����ݡ���Write_Data		д��ʮ�����ݡ���Write_TEN_Data[10]
//����֡�����ֽڳ��ȡ���Modbus_Length	
union Modbus_CRC
{
	uint16_t ALL;
	struct
	{
		uint8_t  low;
		uint8_t  high;    //��λ����
	}nchar;
};
//����8λ��16λ���鹲�����ݵĽ�����
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
	uint8_t		ID;																							//modbusͨѶID
	uint8_t		Function_Code;																	//modbus������
	uint16_t	Start_Address;																	//��ʼ��ַ
	uint16_t	Register_Number;																//�Ĵ�������
	uint8_t		Byte_Number;																		//�ֽڸ���
	union 	Modbus_Transfer_Data 	Read_Data_Array[10];				//����������
	union 	Modbus_Transfer_Data 	Write_Data_Array[10];				//д��������
	uint8_t 	Read_To_Send[20];																//��ȡ���ڷ��͵�����
	union 	Modbus_CRC  	CRC_RX_Data;												//���ܵ���У����
	union 	Modbus_CRC  	CRC_Self_Data;											//�Խ������ݼ���õ���У����
	union 	Modbus_CRC		CRC_TX_Data;												//׼�����͵�У����
	uint8_t		Error;																					//modbus�����־
	uint16_t	Write_Data;																			//д��һ��������
	uint8_t		Const_Columns_Number;														//��ȡ��ǰ������������
	uint8_t		Test;
}MODBUS;

//ModbusͨѶ
extern uint8_t Modbus_Length_In;
extern uint16_t Modbus_Length_Out;
extern MODBUS Modbus;
extern uint8_t Modbus_Buffer[30];

uint8_t Modbus_Process(uint8_t *data_in,uint8_t length_in,uint8_t *data_out,uint16_t* length_out);
uint16_t Modbus_CRC_Data(uint8_t *data,uint16_t length);



#endif
