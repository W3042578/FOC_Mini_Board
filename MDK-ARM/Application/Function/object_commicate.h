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


//ͨѶ������Ŀ
extern uint16_t Commicate_Data_All_Number;
extern const commicate_code Commicate_Code[];


uint16_t Get_Communicte_Data_Number(void);
void Commicate_Data_Init(void);

#endif
