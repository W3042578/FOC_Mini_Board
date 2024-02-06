
#ifndef __PARAMETER_H
#define __PARAMETER_H


#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "foc.h" 

#define MT6815
#define Speed_Transfer	6.5536			//�ٶ�ת������  1rpm -> 1 dec_diff/1ms = 6.5536


typedef  struct
{
		uint8_t 	Work_Model;							//1:У׼  2:ռ�ձ�  3:��ѹ����  4:������ 5:�ٶȻ� 6:λ�û� 7:�ٶȻ��޸� 
		uint8_t 	PWM_Enable;							//0:PWM�ر�ʹ��		1:PWM��ʼʹ��				
		uint8_t 	Energency_Stop;					//0����ʹ�ü�ͣ 1:������ͣ
		uint8_t 	Work_Direction;					//0:��ת		1:��ת
	
		uint8_t		Open_Loop_Voltage;			//������ѹ
		uint8_t 	Max_Voltage;						//����ѹ����
		uint8_t		Angle_Initial_Voltage;	//����������У���ͳ�ʼλ������Ud��ѹ
		uint8_t   Number_Angle_Offest;		//��ʼ��У���ۼӴ���= 2��n�η�
		uint8_t		Clear_Position;					//���õ�ǰλ��Ϊ0
	
		uint8_t		Duty_Model_A;						//ռ�ձ�ģʽ��������ֵ
		uint8_t		Duty_Model_B;
		uint8_t		Duty_Model_C;
	
}_Control_Word;



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

union _Work_Status
{
		uint16_t All;
		struct
		{
			uint8_t 	Enable_Status:1;									//PWMʹ��״̬
			uint8_t 	Angle_Offest:1;										//�������͵�����ȡ����ֵ״̬
			uint8_t 	Duty_Model_Status:1;  						//ռ�ձ�ģʽ״̬
			uint8_t		Offest_Encoder:1;								//��������λУ׼
			uint8_t		Direction_Encoder:1;						//�����������ж�
			uint8_t		Linear_Compensation_Encoder:1;	//���������Բ���
			uint8_t		Offest_Current:1;								//��������
			uint8_t		First_Start:1;									//��������
			uint8_t		Ready_On:1;											//������׼������
			uint8_t		Control_Loop_Error:1;						//���ƻ�·����
			uint8_t		Phase_Lose:1;										//�ඪʧ����  ���û�а���ָ�������˶�
			uint8_t		Encoder_Init:1;									//��������������λ�ó��β���
		}bits;	
};

//����״̬����
extern	_Control_Word Control_Word; 
extern	union _Error_Message Error_Message;
extern  union _Work_Status Work_Status;
extern int16_t Number_Encoder_Offest,Number_Encoder_Direction;

//�������ڱ�����ͨѶ�ǶȻ�ȡ ������֤��
extern uint16_t Transfer1[3];


//��ʼ��
void Hardware_Init(void);
void Error_Message_Init(union _Error_Message *Message);
void Work_Status_Init(union _Work_Status *Status);
void Parameter_Init(void);

#endif
