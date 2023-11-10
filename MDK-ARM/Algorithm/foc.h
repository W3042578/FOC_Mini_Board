#ifndef __FOC_H
#define __FOC_H

#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "spi.h"
#include "usart_control.h" 

//��ѧ���� x16 = x65536
//#define _sqrt3_x16		113511    
//#define _sqrt3_3_x16	37837
//#define _2sqrt3_3_x16	75674
//#define _sqrt3_2_x16	56755
//#define _2pi_x16			411774
//#define	_2pi_360_x16	1144




typedef struct _FOC_MOTOR
{
	uint8_t Polar;								//������
	uint16_t Ts;									//���SVPWM��ѹ�ο�ʸ������
	uint16_t Ts_Limit;
	
	int8_t Udc;									//ĸ�ߵ�ѹ
	int32_t Ud,Uq;
	int32_t Ualph,Ubeta;
	int16_t Ia,Ib;
	int16_t Ia_Offect,Ib_Offect;
	int16_t Ialph,Ibeta;
	int16_t Id,Iq;
	uint32_t Umax;							//����pwm����޷�ʵ��100%�����Ƶ�ѹ��Ϊ����ѹ93.75% = 15/16
	uint32_t Uref;							//Ualph��Ubetaֱ�Ӹ����ĺϳɵ�ѹƽ��ֵ
	int32_t U1,U2,U3,m32;						//�������ϵ�ѹ״̬
	uint8_t Sa,Sb,Sc;
	int32_t Tx,Ty;
	uint8_t Sector_Add,Sector_Actual;	//����
	uint16_t temp16;						//����Tx,Ty�Ͳ�����Ts
	
	uint16_t Encoder;						//����������ֵ
	int32_t Encoder_Angle;
	int16_t Encoder_Speed_Angle;			//1msʱ������ٶ�
	int16_t Encoder_Speed_Angle_Loop;		//�ٶȻ��ڲ������ٶȣ����ٶȻ�ͬ����
	int16_t Speed_Filter;
	int16_t Last_Encoder;					//1msʱ������ٶ���
	int16_t Last_Encoder_Loop;				//�ٶȻ��ڲ������ٶ���
	int16_t Speed_Filter_Loop;
	int8_t	Encoder_Tag;				//����������Ч��־
	uint16_t Encoder_Offest;		//����������ֵ
	
	uint32_t Mechanical_Angle;	//��е�Ƕ�
	uint32_t Elecrical_Angle;		//�����Ƕ�
	int16_t Sin_Angle;					//��Ƕ�sin��cosֵ
	int16_t Cos_Angle;
	int32_t Ta,Tb,Tc;						//����������Ŵ�ʱ�䣨1�����Ŵ򿪣����Źرգ�0�����Ŵ򿪣����Źرգ�
	
	

	
}FOC_Motor;

//����ṹ�����
extern FOC_Motor Motor1;
extern uint16_t Tx_Encoder[2];
extern uint16_t Rx_Encoder[2];

void FOC_Motor_Init(FOC_Motor *motor);
uint16_t Get_Angle_MT6813(FOC_Motor *motor);
void Encoder_Offest(FOC_Motor *motor);
void ADC_Current_Offest(FOC_Motor *motor);
void Get_Electrical_Angle(FOC_Motor *motor);

void Clark_Transform(FOC_Motor *motor);
void Park_Transform(FOC_Motor *motor);
void Inverse_Park_Transform(FOC_Motor *motor);
void SVPWM(FOC_Motor *motor);
void FOC_Control(FOC_Motor *motor);


void Set_Motor_Channel_Time(FOC_Motor *motor);

#endif
