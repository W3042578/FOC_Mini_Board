#include "foc.h"
#include "math.h"
#include "tim.h"
#include "spi.h"
#include "usart.h"
#include "stdio.h"
#include "main.h"

#include "encoder.h"
#include "basic_function.h"
#include "parameter.h"



//�������ṹ��Motor1
FOC_Motor Motor1;		

//��������������ͳһΪ16λ�����в��
//����12λ������1A�����Ӧ2048,1V��ѹ����Ϊ2048,���ֵ�λͳһ
//����Clark�任 �ȷ�ֵ�任
void Clark_Transform(FOC_Motor *motor)
{
	motor->Ialph = motor->Ia;
	motor->Ibeta = (591 * motor->Ia + 1182 * motor->Ib)>>10;//591/1024=sqrt(3)/3
}

//����Park�任
void Park_Transform(FOC_Motor *motor)
{
	//����4096��Ϊ�Ƕ�ֵ�ǳ���4096�������ֵ
	motor->Iq = (motor->Ibeta * motor->Cos_Angle - motor->Ialph * motor->Sin_Angle)>>12;
	motor->Id = (motor->Ibeta * motor->Sin_Angle + motor->Ialph * motor->Cos_Angle)>>12;
} 	

//��ѹInverse Park�任
void Inverse_Park_Transform(FOC_Motor *motor)
{
	//����4096��Ϊ�Ƕ�ֵ�ǳ���4096�������ֵ
	motor->Ualph = (motor->Ud * motor->Cos_Angle - motor->Uq * motor->Sin_Angle)>>12; 
	motor->Ubeta = (motor->Ud * motor->Sin_Angle + motor->Uq * motor->Cos_Angle)>>12;
}

//SVPWM��������PWMʱ���������
//4096Ϊ100%ռ�ձ��������
//�ز�Ƶ�� = ���عܴ򿪹ر�Ƶ�ʣ�SVPWM 7��ʽ��  ����Ƶ�� = ���ת��/��60/��������  һ���ز�Ƶ�ʣ�����Ƶ�� = 10 ��1
void SVPWM(FOC_Motor *motor)
{
	uint8_t N_Sector;//�����ж�ת������ֵ
	//�����ж�
	motor->U1 = motor->Ubeta;
	motor->U2 = (3547 * motor->Ualph - 2048 * motor->Ubeta) >>12;  // 3547/4096=sqrt(3)/2
	motor->U3 = (-3547 * motor->Ualph - 2048 * motor->Ubeta) >>12;
	
	if(motor->U1 > 0)
		motor->Sa=1;
	else
		motor->Sa=0;
	if(motor->U2 > 0)
		motor->Sb=1;
	else
		motor->Sb=0;
	if(motor->U3 > 0)
		motor->Sc=1;
	else
		motor->Sc=0;
	N_Sector = motor->Sa + (motor->Sb<<1) + (motor->Sc<<2);
	//������ѹ����������
	switch(N_Sector)
	{
		case 1: 
			motor->Sector = 2;
			motor->Tx = -motor->U2;									
			motor->Ty = -motor->U3;
		break;
		case 2:
			motor->Sector = 6;
			motor->Tx = -motor->U3;
			motor->Ty = -motor->U1;
		break;
		case 3:
			motor->Sector = 1;
			motor->Tx = motor->U2;
			motor->Ty = motor->U1;
		break;
		case 4:
			motor->Sector = 4;
			motor->Tx = -motor->U1;
			motor->Ty = -motor->U2;
		break;
		case 5:
			motor->Sector = 3;
			motor->Tx = motor->U1;
			motor->Ty = motor->U3;
		break;
		case 6:
			motor->Sector = 5;
			motor->Tx = motor->U3;
			motor->Ty = motor->U2;
		break;
		default:
			motor->Sector = 0;
			motor->Tx = 0;
			motor->Ty = 0;
			break;
	}
	//Tx,Tyת��Ϊʱ�������ʹ��Ts��ʾ����1
	motor->Tx = (motor->Tx * 1774 / motor->Udc) >> 9; //1774>>10=1774/1024=sqrt(3) Tx��2048*ʵ�ʵ�ѹֵ����,Udcͬ�����Ŵ�2048,4096*sqrt(3)*Tx/Udc*2048
	motor->Ty = (motor->Ty * 1774 / motor->Udc) >> 9;
	//������ʸ������ʱ�����ƣ����������ƻ�������MTPA
	if((motor->Tx + motor->Ty) > 3932)//�������� 4096Ϊ1  0.96 * 4096 = 3932  �������������������ʱ��
	{
		uint32_t data_32 = motor->Tx + motor->Ty;
		motor->Tx = motor->Tx * 3932 / data_32;
		motor->Ty = motor->Ty * 3932 / data_32;
	}
	//������תΪ��Ts����ֵ��ʾ
	motor->Tx = (motor->Tx * motor->Ts) >> 12;
	motor->Ty = (motor->Ty * motor->Ts) >> 12;
}

//T0��Tx��Ty������������PWM����ֵ
void PWM_Time_Count(FOC_Motor *motor)
{
	//ռ�ձ�ģʽ����PWMֱ�Ӱ��ո���ֵ��������Ƚ�ֵ
	if(Control_Word.Work_Model == 2)
	{
		motor->Ta = 2.56 *(((100 - Control_Word.Duty_Model_A) * motor->Ts)>>8);
		motor->Tb = 2.56 *(((100 - Control_Word.Duty_Model_B) * motor->Ts)>>8);
		motor->Tc = 2.56 *(((100 - Control_Word.Duty_Model_C) * motor->Ts)>>8);
	}
	//��ռ�ձ�ģʽ�������Tx,Ty����PWM����Ƚ�ֵ
	else
	{
		//T0��һ����Ҫ����T7��������ʽ��г���ɷ���THD���  https://blog.csdn.net/weixin_51553819/article/details/121856985
		//ȡT0 = T7 �� ��Tx + Ty��/2 ����PWM 50% ������� 
		switch(motor->Sector)
		{
			case 1:
				motor->Ta = (motor->Ts - motor->Tx - motor->Ty)>>1;//��
				motor->Tb = (motor->Ts + motor->Tx - motor->Ty)>>1;//��
				motor->Tc =	(motor->Ts + motor->Tx + motor->Ty)>>1;//��
			break;
			case 2:
				motor->Ta = (motor->Ts + motor->Tx - motor->Ty)>>1;//��
				motor->Tb = (motor->Ts - motor->Tx - motor->Ty)>>1;//��
				motor->Tc = (motor->Ts + motor->Tx + motor->Ty)>>1;//��
			break;
			case 3:
				motor->Ta = (motor->Ts + motor->Tx + motor->Ty)>>1;//��
				motor->Tb = (motor->Ts - motor->Tx - motor->Ty)>>1;//��
				motor->Tc = (motor->Ts + motor->Tx - motor->Ty)>>1;//��
			break;
			case 4:
				motor->Ta = (motor->Ts + motor->Tx + motor->Ty)>>1;//��
				motor->Tb = (motor->Ts + motor->Tx - motor->Ty)>>1;//��
				motor->Tc = (motor->Ts - motor->Tx - motor->Ty)>>1;//��
			break;
			case 5:
				motor->Ta = (motor->Ts + motor->Tx - motor->Ty)>>1;//��
				motor->Tb = (motor->Ts + motor->Tx + motor->Ty)>>1;//��
				motor->Tc = (motor->Ts - motor->Tx - motor->Ty)>>1;//��
			break;
			case 6:
				motor->Ta = (motor->Ts - motor->Tx - motor->Ty)>>1;//��
				motor->Tb = (motor->Ts + motor->Tx + motor->Ty)>>1;//��
				motor->Tc = (motor->Ts + motor->Tx - motor->Ty)>>1;//��
			break;
			default:
				motor->Ta = motor->Ts >>1;//Ĭ�����50%ռ�ձ�
				motor->Tb = motor->Ts >>1;
				motor->Tc = motor->Ts >>1;
				break;
		}
	}
	//����0PWM���ʱ��ʱ��ģ�鹤��������
	if(motor->Ta == 0)
		motor->Ta = 1;
	if(motor->Tb == 0)
		motor->Tb = 1;
	if(motor->Tc == 0)
		motor->Tc = 1;
}
//����FOC����
void FOC_Control(FOC_Motor *motor)
{ 
	//��ȡ��Ƕ�
	Encoder_To_Electri_Angle(motor);
	//��ȡ��������Iq,Id
	Clark_Transform(motor);
	Park_Transform(motor);
	
	//ģʽ����
	Model_Control(motor);
	if(Control_Word.Work_Model >= 4)//�ջ���ʹ�÷�park�任
		Inverse_Park_Transform(motor);
	if(Control_Word.Work_Model != 2)//2ģʽΪռ�ձ�ģʽ������ҪSVPWM��������ռ�ձ�
		SVPWM(motor);
	PWM_Time_Count(motor);
}	
