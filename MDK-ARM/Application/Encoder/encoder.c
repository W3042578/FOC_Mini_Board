
#include "spi.h"

#include "encoder.h"
#include "parameter.h"
#include "object_commicate.h"

//本文件针对不同编码器作输入转换，最终输入给16位绝对值角度、速度、相对多圈位置数据

//编码器状态位
#define     Encoder_Init_BIT    BIT0        //初始化完成标志
#define     Encoder_Posi_BIT    BIT1        //编码器反向标志

_Encoder Encoder1;


//SPI通讯式编码器变量
_Encoder _MT6813 =
{
    .Type = MT6813,                 //编码器类型
    .Single_Bit = 14,               //单圈位数
    .Multi_Bit = 0,                 //多圈位数
    .Encoder_Status = 2,            //编码器反向
    .Encoder_Com.Tx_Encoder = {0x8300,0x0000},
    .Encoder_Com.Com_Number =2
};
_Encoder _MT6835 =
{
    .Type = MT6835,                 //编码器类型
    .Single_Bit = 17,               //单圈位数
    .Multi_Bit = 8,                 //多圈位数
    .Encoder_Status = 0,            //编码器正向
    .Encoder_Com.Tx_Encoder = {0x8300,0x0000},
    .Encoder_Com.Com_Number =2
};
_Encoder _KTH7815 =
{
    .Type = KTH7815,                //编码器类型
    .Single_Bit = 17,               //单圈位数
    .Multi_Bit = 8,                 //多圈位数
    .Encoder_Status = 0,            //编码器正向
    .Encoder_Com.Tx_Encoder = {0x8300,0x0000},
    .Encoder_Com.Com_Number =2
};

//初始化编码器变量
void Encoder_Init(_Encoder *encoder)
{
    switch (Control_Word.bits.Encoder_Type)
    {
        case MT6813:
            *encoder = _MT6813;
            break;
        case MT6835:
            *encoder = _MT6835;
            break;
        case KTH7815:
            *encoder = _KTH7815;
            break;
        default:
            break;
    }
}
//根据编码器类型判断有无多圈角度数据，输出上一次编码器修正后原始值，并开启下一次的编码器通讯
void Encoder_Get_Angle(_Encoder *encoder)
{
    uint16_t    angle_single;
    uint32_t    angle_multi;
    uint16_t    angle_single_transfer,angle_multi_transfer;
    int32_t     angle_difference;
    uint16_t    *tx_data,*rx_data;
    uint8_t     com_number;

    tx_data = encoder->Encoder_Com.Tx_Encoder;
    rx_data = encoder->Encoder_Com.Rx_Encoder;
    com_number = encoder->Encoder_Com.Com_Number;
    //使用MT6815编码器 编码器正方向与电机正方向相反
    switch (encoder->Type)
    {
        case MT6813:
            angle_single_transfer = ((*(encoder->Encoder_Com.Rx_Encoder) & 0x00FF) << 8) \
            | ((*(encoder->Encoder_Com.Rx_Encoder + 1) & 0XFF00) >> 8);
            angle_single_transfer = (angle_single_transfer >> 2) & 0x3FFF;

            //SPI通讯拉低片选
            HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		    HAL_SPI_TransmitReceive_DMA(&hspi1,(uint8_t*)tx_data,(uint8_t*)rx_data,com_number);  

            break;
        case MT6835:

            break;
        case KTH7815:
            break;
        default:
            break;
    }

    //单圈位置计算
    if(encoder->Single_Bit <= 16)
    {
        //判断编码器方向与电机Iq方向是否相反
        if(!_TEST(&encoder->Encoder_Status,Encoder_Posi_BIT))
            angle_single = angle_single_transfer << (16 - encoder->Single_Bit);
        else
            angle_single = 65536 - (angle_single_transfer << (16 - encoder->Single_Bit)); 
    }
    else
    {
        //判断编码器方向与电机Iq方向是否相反
        if(!_TEST(&encoder->Encoder_Status,Encoder_Posi_BIT))
            angle_single = angle_single_transfer >> (encoder->Single_Bit - 16);
        else
            angle_single = 65536 - (angle_single_transfer >> (encoder->Single_Bit - 16));
        
    }
    
    //多圈位置计算
    if(encoder->Multi_Bit != 0)
    {
        //多圈编码器绝对位置无需增量式计算
        angle_multi = angle_multi_transfer * 65536 + angle_single;
    }
    else
    {
        //单圈编码器多圈位置用增量累加计算
        //初始化单圈位置缓冲
        if(!_TEST(&encoder->Encoder_Status,Encoder_Init_BIT))
        {
            encoder->Encoder_Pulse_Buffer = angle_single;
            _SET(&encoder->Encoder_Status,Encoder_Init_BIT);
        }
        
         angle_difference = angle_single - encoder->Encoder_Pulse_Buffer;
        //对差值范围进行限制，绝对式编码器回环计数
        if(angle_difference >= 32768)
            angle_difference = 65536 - angle_difference;
        if(angle_difference < -32768)
            angle_difference = 65536 + angle_difference;
        encoder->Encode_Position = encoder->Encode_Position + angle_difference;
       
        angle_multi = encoder->Encode_Position;

        encoder->Encoder_Pulse_Buffer = angle_single;
    }


	//输出单圈、多圈位置
	encoder->Encoder_Pulse = angle_single & 0xFFF8;
    encoder->Encoder_Multi_Pulse = angle_multi;
}




