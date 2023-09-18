#ifndef __OBJECT_COMMICATE_H
#define __OBJECT_COMMICATE_H

#include "main.h"

//����		������		����byteλ��		ͨѸ����������ַ		�������ֵ		������Сֵ		���ú���ָ��

//����ṹ��
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
		uint8_t 	Work_Model;							//0:��ѹ����   1:������  2:�ٶȻ�  3:λ�û�  
		uint8_t		Encoder_Offect_Process;	//0:�����б�����У׼ 1;���б�����У׼
		uint8_t 	Enable;									//0:ͣ��		1:����
		uint8_t		Enable_Buffer;					
		uint8_t 	Energency_Stop;					//0����ʹ�ü�ͣ 1:������ͣ
		uint8_t 	Work_Direction;					//0:��ת		1:��ת
		uint8_t		Open_Loop_Voltage;			//������ѹ
		uint8_t 	Max_Voltage;						//������������ѹΪ15V
}_Control_Word;

typedef  struct
{
		uint16_t 	Target_Current;							//Ŀ�����  
		uint16_t 	Target_Speed;								
		uint16_t 	Target_Position;					
	
		uint8_t		Current_PID_Proportion;			//����PID
		uint8_t		Current_PID_Integral;
		uint8_t		Current_PID_Difference;			
	
		uint8_t		Speed_PID_Proportion;				//�ٶ�PID
		uint8_t		Speed_PID_Integral;
		uint8_t		Speed_PID_Difference;				
	
		uint8_t		Positon_PID_Proportion;			//λ��PID	
		uint8_t		Positon_PID_Integral;
		uint8_t		Positon_PID_Difference;			
		
		uint16_t  	Current_Limit;							//�������
		uint16_t	Speed_Limit;
		uint16_t	Position_Limit;
}_Control_Loop;

union _Error_Message
{
		uint16_t	All;
		struct
		{
			uint8_t		ADC_Error:1;						//������������
			uint8_t		Bus_Voltage:1;					//���ߵ�ѹ����	
			uint8_t		Encoder_Status:1;				//������״̬����
			uint8_t		Modbus_Status:1;				//ModbusͨѶ״̬����
			uint8_t		IIC_Status:1;						//IICͨѶ״̬����
			uint8_t		Control_Loop_Error:1;		//���ƻ�·����
			uint8_t		Phase_Lose:1;						//�ඪʧ����  ���û�а���ָ�������˶�
		}bits;
};

//ͨѶ������Ŀ
extern uint16_t Commicate_Data_All_Number;
extern const commicate_code Commicate_Code[];

//ͨѶ����
extern	_Control_Word Control_Data; 
extern	_Control_Loop	Control_Loop;
extern	union _Error_Message Error_Message;



//ͨѶ��������
extern uint16_t test_one;
extern uint16_t test_two;


uint16_t Get_Communicte_Data_Number(void);
void Commicate_Data_Init(void);

#endif
