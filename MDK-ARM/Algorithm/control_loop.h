#ifndef __CONTROL_LOOP_H
#define __CONTROL_LOOP_H

#include "stm32f1xx_hal.h"
#include "stdint.h"

typedef  struct
{
	int32_t 	Target_Current;						//Ŀ��  
	int32_t 	Target_Speed;								
	int32_t 	Target_Position;
	

	uint16_t	Current_Q_Proportion;			//q�����PID
	uint16_t	Current_Q_Integral;
	uint16_t	Current_Q_Difference;
	
	uint16_t	Current_D_Proportion;			//d�����PID
	uint16_t	Current_D_Integral;
	uint16_t	Current_D_Difference;

	uint16_t	Speed_Proportion;					//�ٶ�PI
	uint16_t	Speed_Integral;
		
	uint16_t	Position_Proportion;			//λ��PI
	uint16_t	Position_Integral;		
	uint16_t 	Position_Feedforward;	
	
	int32_t  	Current_Q_Output_Limit;		//�������
	int32_t  	Current_D_Output_Limit;	
	int32_t		Speed_Output_Limit;
	int32_t		Position_Output_Limit;
}_Control_Loop;

typedef  struct
{
	uint16_t 	Proportion;						  
	uint16_t 	Integral;								
	uint16_t 	Difference;
	uint16_t	Antiback;
	
	uint16_t	Error,Last_Error,Expect,Feedback,Feedforward;	
	int32_t		Proportion_Sum;
	int32_t		Integral_Sum;
	int32_t		Difference_Sum;
	int32_t		Output_Sum;
	
	uint32_t  Proportion_Limit;			//�������
	uint32_t	Integral_Limit;
	uint32_t	Difference_Limit;
	uint32_t	Output_limit;
}_PID_Control;

extern	_Control_Loop	Control_Loop;
extern	_PID_Control Current_Q_PID;
extern	_PID_Control Current_D_PID;
extern	_PID_Control Speed_PI;
extern	_PID_Control Position_PI;


void PID_Control_Deal(_PID_Control * PID_Control);
void Control_Loop_Deal(_Control_Loop *Loop);

//PID�ṹ�������ʼ��
void PID_Control_Init(_PID_Control *PID);
//PID���ƻ����������ʼ��
void Control_Loop_Init(_Control_Loop *Loop);
//1ms�жϸ���PID�ṹ��������
void PID_Control_Update(void);

#endif

