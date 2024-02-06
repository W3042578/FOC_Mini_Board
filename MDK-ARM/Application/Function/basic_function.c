
#include "main.h"
#include "math.h"
#include "tim.h"
#include "foc.h"
#include "parameter.h"
#include "control_loop.h"
#include "sin_cos.h"
#include "encoder.h"
#include "basic_function.h"


//���ܲ�&�ײ㽻��
//��Ҫ��ų��ڹ��ܲ���Ҫ�Եײ��޸ĵĺ���


uint16_t Interrupt_Delay,Number_Offest_Count;

//��ȡ�������ǶȲ�ת��Ϊ��Ƕ�
void Encoder_To_Electri_Angle(FOC_Motor *motor)
{
	uint16_t electri_angle;
	uint32_t offest_angle;
	
	//�ǶȻ�ȡ���������ͬ����
	Encoder_Data_Deal(&encoder1); //��ȡ�������Ƕȣ��ٶȣ�λ��
	Start_Encoder_GET(&encoder1); //����������DMA����ȡ��һ��ʹ�ýǶ�
	
	
	//���ԭ��λ�ö���alpha��
	if(encoder1.Encoder_Angle < motor->Initial_Angle_Offset)
			offest_angle = encoder1.Encoder_Angle - motor->Initial_Angle_Offset + 65535;
	else
			offest_angle = encoder1.Encoder_Angle - motor->Initial_Angle_Offset;
	
	//����ٶȲ��������ýǶ�
	if(motor->Speed_Angle > 0)
	{
		offest_angle = offest_angle + motor->Speed_Angle;
		if(offest_angle > 65535)
			offest_angle = offest_angle - 65535;
	}
	else
	{
		if(offest_angle < motor->Speed_Angle)
			offest_angle = offest_angle + 65535 - motor->Speed_Angle;
		else
			offest_angle = offest_angle - motor->Speed_Angle;
	}
	//ʹ�����������ȡ�� t % 2`(n) �ȼ��� t & (2`(n) - 1)
	//�ο�https://blog.csdn.net/lonyw/article/details/80519652
	electri_angle = (motor->Polar * offest_angle) & 0xFFFE;
	//����ȡ��Ƕȶ�Ӧ���Ǻ���ֵ
	motor->Sin_Angle = SIN_COS_TABLE[(electri_angle >> 7)];
	motor->Cos_Angle = SIN_COS_TABLE[((electri_angle >> 7)+128) & 0x1ff];
}

//������У׼ 
//��ȡ��������Ӧalpha����λ�����Ƕ�ֵ
//����ת��������ȡ���Ի���������ݲ��жϱ�����������q���������Ƿ�һ��
//��ȡ��λ�ýǶ�����ֵ
void Get_Initial_Angle_Offest(FOC_Motor *motor)
{
	//��������λУ��
	if(Control_Word.Work_Model == 1 && Work_Status.bits.Angle_Offest != 0)//�жϹ���ģʽ1��Ϊ��λУ׼״̬����У׼
	{
		if(Interrupt_Delay > 0)	//Interrupt_Delay���������
			Interrupt_Delay --;
		else										//Interrupt_DelayΪ�����б������ۼӼ����������ۼ�ָ������
		{
			motor->Initial_Angle_Offset = motor->Initial_Angle_Offset + encoder1.Encoder_Angle;
			Number_Offest_Count --;
		}
		if(Number_Offest_Count == 0)//ָ�������ۼӺ�ƽ�������λУ׼ֵ
		{
			motor->Initial_Angle_Offset = motor->Initial_Angle_Offset >> (Control_Word.Number_Angle_Offest);
			Control_Word.Work_Model = 0;			//У������˳�У��ģʽ���ر�PWMʹ��
			Control_Word.PWM_Enable = 0;
			Work_Status.bits.Angle_Offest = 0;//������У��λ���� ������һ�ν�����λУ׼
		}
	}
}

//��ȡ���������������ֵ  ����ƫ�õ�ѹ
void ADC_Current_Offest(FOC_Motor *motor)
{
	uint32_t Add_ADC_Offect_U,Add_ADC_Offect_V;
	uint16_t	Number_ADC_Offect;
	Work_Status.bits.Offest_Current = 1;
	Add_ADC_Offect_U = Add_ADC_Offect_V = 0;
	Number_ADC_Offect = 32;
	motor->Ia_Offect = 0;				//У׼ֵ���㣬��������ص�����������ֵӰ��ֱ�Ӳɵõ�����
	motor->Ib_Offect = 0;
	
	motor->Ualph = 0;//Ualph = 0V
	motor->Ubeta = 0;
	SVPWM(motor);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->Ta);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->Tb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motor->Tc);	
	
	//ʹ��drv8313
	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);	
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	HAL_Delay(1000);
	//ƽ����С���
	for(uint8_t i = 1;i<=Number_ADC_Offect;i++)
		{
			Add_ADC_Offect_U = Add_ADC_Offect_U - motor->Ia;
			Add_ADC_Offect_V = Add_ADC_Offect_V - motor->Ib;//�˴��ۼ�ʹ�ø��ſ��ǵ�У׼ʱ��ȡ����ֵ�Ǹ���
			HAL_Delay(2);
		}
	motor->Ia_Offect = Add_ADC_Offect_U >> 5;
	motor->Ib_Offect = Add_ADC_Offect_V >> 5;
	motor->Ia = motor->Ib =0;	
}

//���������������
void Dead_Time_Compensate(FOC_Motor *motor)
{
	//���ݻ������������ж�SVPWM��������ռ�ձ�����ʱ�䲹��
	

}

//����ģʽ����
//����������FOC��������
void Model_Control(FOC_Motor *motor)
{
	//���ݿ������жϹ�����
	switch(Control_Word.Work_Model)
	{	
		//У��ģʽ����ʼ�Ƕ�У���ͱ��������Բ���
		case 1:
			motor->Ualph = 2048 * Control_Word.Angle_Initial_Voltage;//Ualph = 3V
			motor->Ubeta = 0;
			if(Work_Status.bits.Angle_Offest == 0)
			{
				Work_Status.bits.Angle_Offest = 1;
				motor->Initial_Angle_Offset = 0;
				Interrupt_Delay = 32;
				Number_Offest_Count = 1<<(Control_Word.Number_Angle_Offest);
			}
			break;
		
		//ռ�ձ�ģʽ�������������ָ����������ռ�ձȣ����ǲ������98%��
		case 2:
			//������ռ�ձ���ֵ������
			if(Control_Word.Duty_Model_A > 96)
			{
				Control_Word.Duty_Model_A = 96;
			}
			if(Control_Word.Duty_Model_B > 96)
			{
				Control_Word.Duty_Model_B = 96;
			}
			if(Control_Word.Duty_Model_C > 96)
			{
				Control_Word.Duty_Model_C = 96;
			}
			break;
		
		//��ѹ����ģʽ���������õ�Uq��Ud��ѹ��������
		case 3:
			motor->Ud = 0;
			motor->Uq = Control_Word.Open_Loop_Voltage * 2048;
			break;
		
		//�ٶȻ�ģ���޸п��ƣ�ģ���ٶ����ӿ����ٶȿ������
		case 7:
			
			break;
		
		//λ�û�ģʽ��PID�������λ�ñջ����
		case 6:
			
			break;
		
		//�ٶȻ�ģʽ��PID�����ٶȱջ����
		case 5:
			
		  break;
			
		//������ģʽ��PID���Ƶ���Iq��Id�ջ����
		case 4:
			
			break;

		//Ĭ��0ģʽ���������
		default:
//			Error_Message.bits.Control_Loop_Error = 1;
			break;
	}
}

//PWMʹ�ܿ���
void Enable_Logic_Control(void)
{
	//����ֹͣ������ֹͣPWM������ر�����ģ�鹤��
	if(Control_Word.Energency_Stop == 1)
	{
		//�ر�����ģ��
		HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);
		//�ر�����PWM���
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
		//PWMʹ��״̬��0
		Work_Status.bits.Enable_Status = 0;
	}
	else
	{
		if(Control_Word.PWM_Enable == 1)
		{
			//��������PWM���
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
			//ʹ������ģ��
			HAL_GPIO_WritePin(Power_Reset_GPIO_Port,Power_Reset_Pin,GPIO_PIN_SET);
			//PWMʹ��״̬��1
			Work_Status.bits.Enable_Status = 1;
		}
		else
		{
			//�ر�����ģ��
			HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);
			//�ر�����PWM���
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			//PWMʹ��״̬��0
			Work_Status.bits.Enable_Status = 0;
		}
	}
}

//STM32 HAL ����PWM�Ƚ�ֵ����
void STM32_HAL_PWM_SET_Compare(FOC_Motor *motor)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->Ta);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->Tb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motor->Tc);
}



