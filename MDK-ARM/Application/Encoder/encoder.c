#include "foc.h"
#include "main.h"
#include "encoder.h"
#include "math.h"
#include "tim.h"
#include "spi.h"
#include "parameter.h"
#include "stdio.h"
#include "object_commicate.h"

//���ļ���Բ�ͬ������������ת�������������16λ����ֵ�Ƕȡ��ٶȡ���Զ�Ȧλ������


//MT6813 14λ����ֵ������
uint16_t Tx_Encoder[2] = {0x8300,0x0000};  //����6813�������շ����� burstģʽ
uint16_t Rx_Encoder[2];
Encoder encoder1;

//��ʼ���������ݵĻ�ȡ
//����������DMA���䣬�������ע������жϺ���ȡ����һ�νǶ�����Ȼ��ʼ���νǶ�DMA��ȡ
void Start_Encoder_GET(Encoder *encoder)
{
	if(encoder->Type == 1)//SPIͨѶ������
	{
			//��ʼDMA����ǰ����ʱ����NSSƬѡ�ţ�Ƭѡ�ŵĹر���DMA�����жϺ�����
			//�ô�DMA����Ƕ������´μ���
			HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);//����Ƭѡ
			HAL_SPI_TransmitReceive_DMA(&hspi1,(uint8_t*)Tx_Encoder,(uint8_t*)Rx_Encoder,2);  
    }

}

//�ɱ��������ݼ���������ٶȲ�����λ�ü���
//�ж����޶�Ȧ�Ƕ�����
void Encoder_Data_Deal(Encoder *encoder)
{
    uint16_t    Angle_Single,Angle_Multi;
		uint16_t    Angle_Transfer;
    int16_t     Angle_Difference;
    //ʹ��MT6815������ �����������������������෴
    #ifdef MT6815
        Angle_Transfer = ((Rx_Encoder[0] & 0x00FF) << 8) | ((Rx_Encoder[1] & 0XFF00) >> 8);
        Angle_Transfer = (Angle_Transfer >> 2) & 0x3FFF;
    #endif 
    if(encoder->Single_Bit <= 16)//������λ������Ϊ16λ����FOC����
//				Angle[0] = Angle_Transfer << (16 - encoder->Single_Bit);
        Angle_Single = 65536 - (Angle_Transfer << (16 - encoder->Single_Bit));//��������������Iq�����෴
    else
        Angle_Single = Angle_Transfer >> (encoder->Single_Bit - 16);
    if(encoder->Multi_Bit != 0)//��Ȧû����λ�����ƣ���MT6813��δʹ��
    {
        Angle_Multi = Angle_Transfer >>16;
    }
	//��Ȧλ�÷�Χ����
	encoder->Encoder_Angle = Angle_Single & 0xFFFF;

    //���������ݲ�ֵ�ۼӵó����λ��
    //�����ϵ�λ������
    if(Work_Status.bits.Encoder_Init == 1)
    {
        encoder->Encoder_Angle_Buffer = encoder->Encoder_Angle;
				Work_Status.bits.Encoder_Init = 0;
    }
    Angle_Difference = encoder->Encoder_Angle - encoder->Encoder_Angle_Buffer;
    encoder->Encode_Position = encoder->Encode_Position + Angle_Difference;
    if(encoder->Encode_Position > 16777216)//24λλ�÷�Χ��������ػ�
    {
         encoder->Encode_Position = encoder->Encode_Position - 33554432;
    }
    else if (encoder->Encode_Position < -16777216)
    {
        encoder->Encode_Position = encoder->Encode_Position + 33554432;
    }
		encoder->Encoder_Angle_Buffer = encoder->Encoder_Angle;
}




