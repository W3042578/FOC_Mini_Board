
#ifndef __BASIC_FUNCTION_H
#define __BASIC_FUNCTION_H



#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "foc.h"

//���ܲ�&�ײ㽻��
//��Ҫ��ų��ڹ��ܲ���Ҫ�Եײ��޸ĵĺ���

//�Ƕ�ת��
void Encoder_To_Electri_Angle(FOC_Motor *motor);

//��ʼ����
void Get_Initial_Angle_Offest(FOC_Motor *motor);
void ADC_Current_Offest(FOC_Motor *motor);

//����
void Dead_Time_Compensate(FOC_Motor *motor);

//ģʽ����
void Model_Control(FOC_Motor *motor);

//ʹ�ܿ����߼�
void Enable_Logic_Control(void);

//STM32 HAL ����PWM�Ƚ�ֵ����
void STM32_HAL_PWM_SET_Compare(FOC_Motor *motor);

#endif

