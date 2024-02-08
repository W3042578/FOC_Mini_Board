
#include "spi.h"

#include "encoder.h"
#include "parameter.h"
#include "object_commicate.h"

//本文件针对不同编码器作输入转换，最终输入给16位绝对值角度、速度、相对多圈位置数据

_Encoder Encoder1;

//MT6813 14位绝对值编码器
uint16_t Tx_Encoder[2] = {0x8300,0x0000};  //定义6813编码器收发数据 burst模式
uint16_t Rx_Encoder[2];


//开始编码器数据的获取
//编码器数据DMA传输，进入电流注入采样中断后先取出上一次角度数据然后开始本次角度DMA获取
void Start_Encoder_GET(_Encoder *encoder)
{
	if(encoder->Type == 1)//SPI通讯编码器
	{
		//开始DMA发送前数据时拉低NSS片选脚，片选脚的关闭在DMA传输中断函数中
		//该次DMA传输角度用于下次计算
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);//拉低片选
		HAL_SPI_TransmitReceive_DMA(&hspi1,(uint8_t*)Tx_Encoder,(uint8_t*)Rx_Encoder,2);  
    }
}

//由编码器数据计算编码器速度并进行位置计数
//判断有无多圈角度数据
void Encoder_Data_Deal(_Encoder *encoder)
{
    uint16_t    Angle_Single,Angle_Multi;
    uint16_t    Angle_Transfer;
    int32_t     Angle_Difference;
    //使用MT6815编码器 编码器正方向与电机正方向相反
    #ifdef MT6815
        Angle_Transfer = ((Rx_Encoder[0] & 0x00FF) << 8) | ((Rx_Encoder[1] & 0XFF00) >> 8);
        Angle_Transfer = (Angle_Transfer >> 2) & 0x3FFF;
    #endif 
    if(encoder->Single_Bit <= 16)//编码器位数调整为16位进行FOC计算
//				Angle[0] = Angle_Transfer << (16 - encoder->Single_Bit);
        Angle_Single = 65536 - (Angle_Transfer << (16 - encoder->Single_Bit));//编码器方向与电机Iq方向相反
    else
        Angle_Single = Angle_Transfer >> (encoder->Single_Bit - 16);
    if(encoder->Multi_Bit != 0)//多圈没有作位数限制，在MT6813中未使用
    {
        Angle_Multi = Angle_Transfer >>16;
    }
	//单圈位置范围限制
	encoder->Encoder_Angle = Angle_Single & 0xFFFF;

    //编码器数据差值累加得出相对位置
    //初次上电位置清零
    if(Work_Status.bits.Encoder_Init == 1)
    {
        encoder->Encoder_Angle_Buffer = encoder->Encoder_Angle;
	Work_Status.bits.Encoder_Init = 0;
    }
	
    Angle_Difference = encoder->Encoder_Angle - encoder->Encoder_Angle_Buffer;
    //对差值范围进行限制，绝对式编码器回欢计数
    if(Angle_Difference > 32768)
	    Angle_Difference = 65536 - Angle_Difference;
    if(Angle_Difference < -32768)
	    Angle_Difference = 65536 + Angle_Difference;
    encoder->Encode_Position = encoder->Encode_Position + Angle_Difference;
    if(encoder->Encode_Position > 16777216)//24位位置范围，超过则回环
    {
         encoder->Encode_Position = encoder->Encode_Position - 33554432;
    }
    else if (encoder->Encode_Position < -16777216)
    {
        encoder->Encode_Position = encoder->Encode_Position + 33554432;
    }
    encoder->Encoder_Angle_Buffer = encoder->Encoder_Angle;
}




