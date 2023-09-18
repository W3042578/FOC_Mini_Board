#ifndef __CONTROL_LOOP_H
#define __CONTROL_LOOP_H

#include "stm32f1xx_hal.h"
#include "stdint.h"


typedef struct _FOC_PID
{
	uint16_t Kp,Ki,Kd;																		//PID����
	int32_t Expect,Feed_Back,Error,Last_Error;						//���������������ϴ����
	int32_t Integral,Different,Output;										//���֡�΢�֡����
	
	int32_t Error_Low_Limit,Error_Upper_Limit;						//���������
	int32_t Integral_Low_Limit,Integral_Upper_Limit;			//����������
	int32_t Different_Low_Limit,Different_Upper_Limit;		//΢��������
	int32_t Output_Low_Limit,Output_Upper_Limit;					//���������
	
}FOC_PID;


void Current_PID_Init(FOC_PID *foc_pid);
void Speed_PID_Init(FOC_PID *foc_pid);
void Position_PID_Init(FOC_PID *foc_pid);

void PID_Control(FOC_PID* foc_pid);


extern FOC_PID	FOC_PID_Current_Iq;									//����ȫ�ֵ�����Iq PID�ṹ��
extern FOC_PID	FOC_PID_Current_Id;									//����ȫ�ֵ�����Id PID�ṹ��
extern FOC_PID	FOC_PID_Speed;											//����ȫ���ٶȻ�PID�ṹ��
extern FOC_PID	FOC_PID_Position;										//����ȫ��λ�û�PID�ṹ��
extern uint16_t Loop_Time_Count;									//���ƻ�·ʱ����㣬1Dec = 62.5us

#endif

