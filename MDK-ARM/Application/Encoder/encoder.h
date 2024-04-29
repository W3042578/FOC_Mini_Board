
#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f1xx_hal.h"


typedef struct _encoder_com
{
	uint16_t Rx_Encoder[4];
	uint16_t Tx_Encoder[4];
	uint8_t	Com_Number;
}_Encoder_Com;

typedef struct _encoder
{
	uint8_t			Type;					//编码器类型
	uint8_t			Single_Bit;				//单圈位数
	uint8_t			Multi_Bit;				//多圈位数
	uint16_t		Encoder_Pulse;			//编码器16位值
	uint32_t		Encoder_Multi_Pulse;	//编码器多圈位置
	uint16_t		Encoder_Pulse_Buffer;	//编码器上一次值
	uint32_t		Encode_Position;		//编码器位置数据，包含多圈计数
	uint32_t 		Encoder_Offest;			//编码器修正值
	uint8_t			Encoder_Status;			//编码器状态
	_Encoder_Com	Encoder_Com;			//编码器通讯数据
}_Encoder;

//编码器类型
enum encoder_type
{
	MT6813 = 1,
	MT6835 = 2,
	KTH7815 = 3
};

//编码器变量
extern  _Encoder Encoder1;;

void Encoder_Init(_Encoder *encoder);
void Encoder_Get_Angle(_Encoder *encoder);

#endif

