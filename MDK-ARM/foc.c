#include "foc.h"
#include "math.h"
#include "tim.h"
#include "spi.h"
#include "usart.h"
#include "stdio.h"
#include "sin_cos.h"
#include "main.h"
#include "usart_control.h"
#include "control_loop.h"
#include "object_commicate.h"

//FOC_Motor �ṹ�������ʼ��
//7������
//���ù���ʱ��Ts = 2*(2+1)*(1499+1)/72M = 125us  ���Ķ���ģʽx2
//ĸ�ߵ�ѹΪ 12V
//��������Ч�ʣ�������ȫ��ת��Ϊ�������� ������λ����
//�Ŵ���Ϊ 0XFFFF = 65535+1 16λ
//���ɼ���12����*4��Ϊ�����16λ���ݣ�ע��������ݷ�Χ��Ҫ���
//ȡia��ibΪ14λ���ݣ���id��iq��Ud��Uq,Umax,Uref��Ϊ14λ�������
void FOC_Motor_Init(FOC_Motor *motor)
{
	motor->Polar = 11;
	motor->Ts = 1480;
	motor->Udc = 12;
	motor->Last_Encoder = 0;
	motor->Ts_Limit = motor->Ts ;
}
//����Clark�任
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
//SVPWM����ʱ��
//�ο��� https://blog.csdn.net/hducollins/article/details/79260176
//�ο��� https://blog.csdn.net/qq_41610710/article/details/120512746
void SVPWM(FOC_Motor *motor)
{
	//��ֵ���ƣ���������Ʋ���Բ��,0.9375Ϊ���ǵ������ʱ�������ž��е�ͨ�����������	
	motor->Uref = (motor->Ualph * motor->Ualph + motor->Ubeta * motor->Ubeta)>>14;
	if(motor->Uref > 0.33 * ((motor->Udc * motor->Udc)<<14))
	{
		motor->Ualph = (motor->Ualph * motor->Udc *motor->Udc)/motor->Uref;//�������Ƶ���ԲӦ���ǿ�����ȡ�������˴���ʡ����ʹ��ƽ��ȡ���������С
		motor->Ubeta = (motor->Ubeta * motor->Udc *motor->Udc)/motor->Uref;
	}
	
	//�����ж�
	motor->U1 = motor->Ubeta;
	motor->U2 = (3547 * motor->Ualph - 2048 * motor->Ubeta) >>12;  // 3547/4096=sqrt(3)/2
	motor->U3 = (-3547 * motor->Ualph - 2048 * motor->Ubeta) >>12;
	
	if(motor->U1>0)
		motor->Sa=1;
	else
		motor->Sa=0;
	if(motor->U2>0)
		motor->Sb=1;
	else
		motor->Sb=0;
	if(motor->U3>0)
		motor->Sc=1;
	else
		motor->Sc=0;
	motor->Sector_Add = motor->Sa + (motor->Sb<<1) + (motor->Sc<<2);
	
	//����ʸ������ʱ��,ע��motor->Tx��motor->Ty�Ⱥ�����Ӧ�ĵ�ѹʸ��
	motor->m32 =(1774 * motor->Ts/motor->Udc) >>10;// motor->Udc û�зŴ�1774/1024=sqrt(3) ��m�Ŵ�14λʹm��U1,U2,U3ͬ�Ŵ�λ
	switch(motor->Sector_Add)
	{
		case 1: 
			motor->Sector_Actual = 2;
			motor->Tx = (-motor->U2 * motor->m32) >>14;									//����14λ�ȼ��ڳ���16384
			motor->Ty = (-motor->U3 * motor->m32) >>14;
		break;
		case 2:
			motor->Sector_Actual = 6;
			motor->Tx = (-motor->U3 * motor->m32) >>14;
			motor->Ty = (-motor->U1 * motor->m32) >>14;
		break;
		case 3:
			motor->Sector_Actual = 1;
			motor->Tx = (motor->U2 * motor->m32) >>14;
			motor->Ty = (motor->U1 * motor->m32) >>14;
		break;
		case 4:
			motor->Sector_Actual = 4;
			motor->Tx = (-motor->U1 * motor->m32) >>14;
			motor->Ty = (-motor->U2 * motor->m32) >>14;
		break;
		case 5:
			motor->Sector_Actual = 3;
			motor->Tx = (motor->U1 * motor->m32) >>14;
			motor->Ty = (motor->U3 * motor->m32) >>14;
		break;
		case 6:
			motor->Sector_Actual = 5;
			motor->Tx = (motor->U3 * motor->m32) >>14;
			motor->Ty = (motor->U2 * motor->m32) >>14;
		break;
		default:
			motor->Sector_Actual = 0;
			motor->Tx = 0;
			motor->Ty = 0;
			break;
	}
	if((motor->Tx + motor->Ty)>motor->Ts_Limit)
	{
		motor->temp16 =motor->Tx + motor->Ty;
		motor->Tx = motor->Tx * motor->Ts_Limit / motor->temp16;
		motor->Ty = motor->Ty * motor->Ts_Limit / motor->temp16;
	}
		switch(motor->Sector_Actual)
	{
		case 1:
			motor->Ta =(motor->Ts + motor->Tx + motor->Ty)>>1;	//����1λ�ȼ��ڳ���2
			motor->Tb =(motor->Ts - motor->Tx + motor->Ty)>>1;
			motor->Tc =(motor->Ts - motor->Tx - motor->Ty)>>1;
		break;
		case 2:
			motor->Ta =(motor->Ts - motor->Tx + motor->Ty)>>1;
			motor->Tb =(motor->Ts + motor->Tx + motor->Ty)>>1;
			motor->Tc =(motor->Ts - motor->Tx - motor->Ty)>>1;
		break;
		case 3:
			motor->Ta =(motor->Ts - motor->Tx - motor->Ty)>>1;
			motor->Tb =(motor->Ts + motor->Tx + motor->Ty)>>1;
			motor->Tc =(motor->Ts - motor->Tx + motor->Ty)>>1;
		break;
		case 4:
			motor->Ta =(motor->Ts - motor->Tx - motor->Ty)>>1;
			motor->Tb =(motor->Ts - motor->Tx + motor->Ty)>>1;
			motor->Tc =(motor->Ts + motor->Tx + motor->Ty)>>1;
		break;
		case 5:
			motor->Ta =(motor->Ts - motor->Tx + motor->Ty)>>1;
			motor->Tb =(motor->Ts - motor->Tx - motor->Ty)>>1;
			motor->Tc =(motor->Ts + motor->Tx + motor->Ty)>>1;
		break;
		case 6:
			motor->Ta =(motor->Ts + motor->Tx + motor->Ty)>>1;
			motor->Tb =(motor->Ts - motor->Tx - motor->Ty)>>1;
			motor->Tc =(motor->Ts - motor->Tx + motor->Ty)>>1;
		break;
		default:
			motor->Ta =0;
			motor->Tb =0;
			motor->Tc =0;
			break;
	}
}

//������·��ʱ��ccr������ֵ
//�ο��߼���ʱ��������·����������pwm ��https://blog.csdn.net/wsq_666/article/details/126555569
void Set_Motor_Channel_Time(FOC_Motor *motor)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->Ta);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->Tb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motor->Tc);
	
}

//��ȡ�������Ƕ�ֵ
//��λ������ֵ 2~14   14λ������
uint16_t Get_Angle_MT6813(FOC_Motor *motor)
{
	uint16_t Angle;
	uint16_t Angle_Transfer[2];
	uint16_t Tx_Data[2];
	uint8_t Rx_Data[2];
	uint8_t * Tx_Data1 = (uint8_t *)&Tx_Data[0];
	uint8_t * Tx_Data2 = (uint8_t *)&Tx_Data[1];
	Tx_Data[0] = 0x8300;//MT6813��ȡ�Ĵ�����ַ����λ��ǰ�ȷ��ͣ�ע��16λ����
	Tx_Data[1] = 0X8400;//MT6813��ȡ�Ĵ�����ַ����λ��ǰ�ȷ���
	//��һ���Ĵ�������
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //SPIƬѡ���õ�
	HAL_SPI_Transmit(&hspi1,Tx_Data1,1,10);
	HAL_SPI_Receive(&hspi1,&Rx_Data[0],1,10);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	//�ڶ����Ĵ�������
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,Tx_Data2,1,10);
	HAL_SPI_Receive(&hspi1,&Rx_Data[1],1,10);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
//	Transfer1 = Rx_Data[0];
	//�������ݺϲ��ó�����ֵ,��������������ǿ��״̬�źţ�1Ϊ�ų�ǿ�Ȳ��㣬0Ϊ��������
	Angle_Transfer[0] = (((uint16_t)Rx_Data[0]) << 6) & 0x3FC0;   //��������6λ
	(*motor).Encoder_Tag =(uint8_t)((0x0002 & Rx_Data[1]) >>1);  		//��ȡ�ڶ�λ��״̬
	Angle_Transfer[1] = (((uint16_t)Rx_Data[1]) >> 2) & 0x003F;			//��������2λ
	Angle = Angle_Transfer[0] + Angle_Transfer[1];
	
	return Angle;
}
//��ȡ����������ֵ
void Encoder_Offest(FOC_Motor *motor)
{
	uint32_t Add_Encoder_Offest;
	uint16_t Number_Encoder_Offest;
	Number_Encoder_Offest = 32;
	Add_Encoder_Offest	= 0;
	
 	motor->Ualph = 49152;//Ualph = 3V
	motor->Ubeta = 0;
	SVPWM(motor);
	Set_Motor_Channel_Time(motor);
	
	//ʹ��drv8313
	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);	
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	HAL_Delay(1000);
	//ƽ����С���
	for(uint8_t i = 1;i<=Number_Encoder_Offest;i++)
		{
			Add_Encoder_Offest = Add_Encoder_Offest + 16384 - Get_Angle_MT6813(motor);
		}
		
	motor->Encoder_Offest = Add_Encoder_Offest>>5;
		
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	
	//�ر�ʹ��drv8313
	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);	
}
//��ȡ���������������ֵ  ����ƫ�õ�ѹ
void ADC_Current_Offest(FOC_Motor *motor)
{
	uint32_t Add_ADC_Offect_U,Add_ADC_Offect_V;
	uint16_t	Number_ADC_Offect;
	
	Add_ADC_Offect_U = Add_ADC_Offect_V = 0;
	Number_ADC_Offect = 32;
	motor->Ia_Offect = 0;				//У׼ֵ���㣬��������ص�����������ֵӰ��ֱ�Ӳɵõ�����
	motor->Ib_Offect = 0;
	
	motor->Ualph = 0;//Ualph = 0V
	motor->Ubeta = 0;
	SVPWM(motor);
	Set_Motor_Channel_Time(motor);
	
	//ʹ��drv8313
	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);	
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	HAL_Delay(100);
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
		
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
		
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	//�ر�ʹ��drv8313
	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);	
}
//��ȡ���������ǶȲ�ת��Ϊsin��cosֵ�����ݱ����������Ƕȼ����ٶ�
void Get_Electrical_Angle(FOC_Motor *motor)
{
	motor->Encoder = 16384 - Get_Angle_MT6813(motor);
	if(motor->Encoder - motor->Encoder_Offest < 0)
		motor->Encoder = motor->Encoder - motor->Encoder_Offest + 16384;
	else
		motor->Encoder = motor->Encoder - motor->Encoder_Offest;
	//��ȡ������е�Ƕȶ�Ӧ�ĵ����Ƕȣ���λ��������
	motor->Mechanical_Angle = (360*motor->Encoder)>>14;
	motor->Elecrical_Angle = (motor->Polar * motor->Mechanical_Angle)%360;
	//�ɽǶ�ֵ��ȡ��Ӧsin��cos����4096ֵ
	motor->Sin_Angle = Get_Sin(motor->Elecrical_Angle);
	motor->Cos_Angle = Get_Cos(motor->Elecrical_Angle);

	//�����ٶ�  ���ڽǶȼ�ȥ��ȥ�Ƕ�
	motor->Encoder_Speed_Angle_Loop = motor->Encoder - motor->Last_Encoder_Loop;
	if(Control_Data.Work_Direction == 0)//��ת�ٶȱ�Ϊ��
    {
      if(motor->Encoder_Speed_Angle_Loop < 0)
        motor->Encoder_Speed_Angle_Loop = 16384 + motor->Encoder_Speed_Angle_Loop;
    }
	else                 	            //��ת�ٶȱ�Ϊ��
    {
      if(Motor1.Encoder_Speed_Angle > 0)
		  	motor->Encoder_Speed_Angle_Loop = motor->Encoder_Speed_Angle_Loop - 16384;
    }
	motor->Speed_Filter_Loop = (motor->Speed_Filter_Loop + 3 * motor->Encoder_Speed_Angle_Loop)>>2;
    //�˴��ٶ���Ϊ��һ�εĹ����ٶ�
	motor->Last_Encoder_Loop = motor->Encoder_Speed_Angle_Loop;
}
//FOC����
//���ݿ�����ģʽѡ����ģʽ�͹�����������ѹ���ٶȡ�λ��Ҫ�󣬲��ж��Ƿ�ʹ��
void FOC_Control(FOC_Motor *motor)
{
	Get_Electrical_Angle(motor);
	switch(Control_Data.Work_Model)
	{
		case 0x00:																			//��������
			if(Control_Data.Work_Direction)
			motor->Uq = -16384 * Control_Data.Open_Loop_Voltage;
			else
			motor->Uq = 16384 * Control_Data.Open_Loop_Voltage;
			motor->Ud = 0;
			break;
			
		case 0x01:																			//������
			Clark_Transform(motor);
			Park_Transform(motor);
		
			//Iq PID
			FOC_PID_Current_Iq.Expect = Control_Loop.Target_Current;
			FOC_PID_Current_Iq.Feed_Back = (*motor).Iq;
			PID_Control(&FOC_PID_Current_Iq);
			(*motor).Uq = FOC_PID_Current_Iq.Output;  
		
			//Id PID
			FOC_PID_Current_Id.Expect = 0;
			FOC_PID_Current_Id.Feed_Back = (*motor).Id;
			PID_Control(&FOC_PID_Current_Id);
			(*motor).Ud = FOC_PID_Current_Id.Output<<3;
			break;
		
		case 0x10:																			//�ٶȻ�
			if(Loop_Time_Count == 0)
			{
				Loop_Time_Count = 2;
				FOC_PID_Speed.Expect = Control_Loop.Target_Speed;
				//�ٶȲ����ж��߼�:��8kƵ�����ܹ��ֱ���ٶȱ仯��ʹ��8k�ٶȻ���ȡ���ݼ����ٶ�ֵ�����������ʹ��1k��ʱ����ȡ���ݼ����ٶ�ֵ
				//������������ٶȼ������Ŵ�
				if(Motor1.Encoder_Speed_Angle_Loop < 20)
					FOC_PID_Speed.Feed_Back = (1875 * (*motor).Encoder_Speed_Angle)>>9; //1875/512 = 60*1000/16384  1msת��Ϊrpm��λ
				else
					FOC_PID_Speed.Feed_Back = (1875 * (*motor).Encoder_Speed_Angle_Loop)>>6; //8*1875/512 = 8*60*1000/16384  0.125msת��Ϊrpm��λ
				PID_Control(&FOC_PID_Speed);
			}
			Clark_Transform(motor);
			Park_Transform(motor);
		
			//Iq PID
			FOC_PID_Current_Iq.Expect = FOC_PID_Speed.Output;
			FOC_PID_Current_Iq.Feed_Back = (*motor).Iq;
			PID_Control(&FOC_PID_Current_Iq);
			(*motor).Uq = FOC_PID_Current_Iq.Output;
		
			//Id PID
			FOC_PID_Current_Id.Expect = 0;
			FOC_PID_Current_Id.Feed_Back = (*motor).Id;
			PID_Control(&FOC_PID_Current_Id);
			(*motor).Ud = FOC_PID_Current_Id.Output;
			
			Loop_Time_Count--;
			break;
		case 0x11:																		//λ�û�
			if(Loop_Time_Count == 0)
			{
				Loop_Time_Count = 4;
				FOC_PID_Position.Expect = Control_Loop.Target_Position;
				FOC_PID_Position.Feed_Back = (*motor).Encoder;
				PID_Control(&FOC_PID_Position);
			}
			FOC_PID_Speed.Expect = FOC_PID_Position.Output;
			FOC_PID_Speed.Feed_Back = (1875 * (*motor).Encoder_Speed_Angle)>>9; //1875/512 = 60*1000/16384  ת��Ϊrpm��λ
			PID_Control(&FOC_PID_Speed);
		
			Clark_Transform(motor);
			Park_Transform(motor);
			
			//Iq PID
			FOC_PID_Current_Iq.Expect = FOC_PID_Speed.Output;
			FOC_PID_Current_Iq.Feed_Back = (*motor).Iq;
			PID_Control(&FOC_PID_Current_Iq);
			(*motor).Uq = FOC_PID_Current_Iq.Output;
		
			//Id PID
			FOC_PID_Current_Id.Expect = 0;
			FOC_PID_Current_Id.Feed_Back = (*motor).Id;
			PID_Control(&FOC_PID_Current_Id);
			(*motor).Ud = FOC_PID_Current_Id.Output;
			Loop_Time_Count--;
			
			break;
		default:
			Error_Message.bits.Control_Loop_Error = 1;
			break;
	}
	Inverse_Park_Transform(motor);
	SVPWM(motor);
	Set_Motor_Channel_Time(motor);
}
