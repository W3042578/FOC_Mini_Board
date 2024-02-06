
#ifndef __ENCODER_H
#define __ENCODER_H
#include "foc.h"
#include "stm32f1xx_hal.h"


typedef struct _Encoder
{
	uint8_t		Type;							//����������
	uint8_t		Multi_Bit;						//��Ȧλ��
	uint8_t		Single_Bit;						//��Ȧλ��
	uint16_t	Encoder_Angle;					//������16λֵ
	uint16_t	Encoder_Angle_Buffer;			//��������һ��ֵ
	int32_t		Encode_Position;				//������λ�����ݣ�������Ȧ����
	int16_t 	Encoder_Speed_Angle;			//1ms�������ٶ�
	int16_t 	Encoder_Speed_Angle_Buffer;		//��һ��1ms�������ٶ�
	int16_t 	Encoder_Speed_Angle_Loop;		//�ٶȻ��������ٶ�
	int16_t 	Encoder_Speed_Angle_Loop_Buffer;//��һ���ٶȻ��������ٶ�
	int16_t 	Speed_Filter;					//�ٶ��˲�����λ
	int16_t		Last_Encoder_Loop;				//�ٶȻ��ڲ������ٶ���
	int16_t 	Speed_Filter_Loop;              //�ٶȻ��ٶȼ����˲�
	uint32_t 	Encoder_Offest_Data;		        	//����������ֵ
	uint8_t		Encoder_Direction;				//������������q�᷽����ͬΪ0���෴Ϊ1
	uint16_t 	Encoder_Direction_Position;		//�����������ж���λ��
}Encoder;

extern  uint16_t Tx_Encoder[2];
extern  uint16_t Rx_Encoder[2];

//����������
extern  Encoder encoder1;

void Start_Encoder_GET(Encoder *encoder);
void Encoder_Data_Deal(Encoder *encoder);

#endif

