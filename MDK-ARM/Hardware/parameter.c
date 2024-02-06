#include "parameter.h"

#include "foc.h"
#include "encoder.h"
#include "object_commicate.h"
#include "control_loop.h"

union _Error_Message Error_Message;		//������Ϣָʾλ
union _Work_Status Work_Status;				//����״ָ̬ʾλ

_Control_Word Control_Word; 					//ͨ����������Ʊ�������
_Control_Loop	Control_Loop;						//���ƻ�·����

int16_t Number_Encoder_Direction;			//����

//Ӳ������
void Hardware_Init(void)
{
	//���
	Motor1.Polar = 11;
	Motor1.Udc = 12;						//ĸ�ߵ�ѹΪ 12V
		
	//������
	encoder1.Type = 1; 					//SPIͨѶ������
	encoder1.Single_Bit = 14;		//14λ��Ȧ
	encoder1.Multi_Bit = 0;			//��ȦλΪ0
	
	//MCU����
	//���ù���ʱ��Ts = 2*(2+1)*(1499+1)/72M = 125us  ���Ķ���ģʽx2
	Motor1.Ts = 2249;//16kƵ���¶�ʱ������ֵ  ����2249  
}



//������
void Control_Word_Init(_Control_Word *Data)
{
	Data->Work_Model = 0;							//1:У׼  2:ռ�ձ�  3:��ѹ����  4:������ 5:�ٶȻ� 6:λ�û� 7:�ٶȻ��޸�
	Data->PWM_Enable = 0;							//0:PWM�ر�ʹ��		1:PWM��ʼʹ��
	Data->Energency_Stop = 0;					//1:�������ֹͣ
	Data->Work_Direction = 0;					//0:��ǰ���� 1:��ǰ����
	Data->Open_Loop_Voltage = 0;			//������ѹ
	Data->Max_Voltage = 16;						//����ѹ����
	Data->Angle_Initial_Voltage = 5;	//����������У���ͳ�ʼλ������Ud��ѹ
	Data->Number_Angle_Offest = 5;		//���� = 2��n�η�
	Data->Clear_Position = 0;					//���õ�ǰλ��Ϊ0
	
	Data->Duty_Model_A = 50;					//ռ�ձ�ģʽ�����ʼֵ
	Data->Duty_Model_B = 50;
	Data->Duty_Model_C = 50;
}

//������
void Error_Message_Init(union _Error_Message *Message)
{
	Message->All = 0;
}	

//����״̬��
void Work_Status_Init(union _Work_Status *Status)
{
	Status->All = 0;
}

//������ʼ��
void Parameter_Init(void)
{
	//Ӳ��������ʼ��
	Hardware_Init();
	
	//modbusͨѶ���ݳ�ʼ��
	Commicate_Data_Init();
	
	//���ƻ�������ʼ��
	Control_Loop_Init(&Control_Loop);
	
	//�����ֳ�ʼ��
	Control_Word_Init(&Control_Word);
	
	//����״̬��ʼ��
	Error_Message_Init(&Error_Message);
	
	//����״̬��ʼ��
	Work_Status_Init(&Work_Status);
	
}
