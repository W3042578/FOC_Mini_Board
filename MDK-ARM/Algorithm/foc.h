#ifndef __FOC_H
#define __FOC_H

#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "spi.h"
#include "usart_control.h" 


typedef struct _FOC_MOTOR
{
	uint8_t Polar;								//������
	uint16_t Ts;								//���SVPWM��ѹ�ο�ʸ������
	
	int8_t Udc;									//ĸ�ߵ�ѹ
	int32_t Ud,Uq;
	int32_t Ualph,Ubeta;
	int16_t Ia,Ib;
	int16_t Ia_Offect,Ib_Offect;
	int16_t Ialph,Ibeta;
	int16_t Id,Iq;
	uint32_t Umax;							//pwm����޷�ʵ��100%ռ�ձ�
	uint32_t Uref;							//Ualph��Ubetaֱ�Ӹ����ĺϳɵ�ѹƽ��ֵ
	int32_t U1,U2,U3,m32;					//�������ϵ�ѹ״̬
	uint8_t Sa,Sb,Sc;
	int32_t Tx,Ty;
	uint8_t Sector;		//����
		
	uint32_t Mechanical_Angle;				//��е�Ƕ�
	uint32_t Elecrical_Angle;				//�����Ƕ�
	uint32_t Initial_Angle_Offset;	//��ʼλ�������ǣ�����alpha��
	int32_t	 Speed_Angle;						//ת�ٲ����Ƕ�
	int16_t Sin_Angle;						//��Ƕ�sin��cosֵ
	int16_t Cos_Angle;
	int32_t Ta,Tb,Tc;						//����������Ŵ�ʱ�䣨1�����Ŵ򿪣����Źرգ�0�����Ŵ򿪣����Źرգ�
	
	uint8_t Direction;						//�����������
}FOC_Motor;

//����ṹ�����
extern FOC_Motor Motor1;


void Clark_Transform(FOC_Motor *motor);
void Park_Transform(FOC_Motor *motor);
void Inverse_Park_Transform(FOC_Motor *motor);
void SVPWM(FOC_Motor *motor);
void PWM_Time_Count(FOC_Motor *motor);
void FOC_Control(FOC_Motor *motor);



#endif
