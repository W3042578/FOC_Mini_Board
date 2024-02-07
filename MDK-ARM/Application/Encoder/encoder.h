
#ifndef __ENCODER_H
#define __ENCODER_H
#include "foc.h"
#include "stm32f1xx_hal.h"


typedef struct _Encoder
{
	uint8_t		Type;					//编码器类型
	uint8_t		Multi_Bit;				//多圈位数
	uint8_t		Single_Bit;				//单圈位数
	uint16_t	Encoder_Angle;				//编码器16位值
	uint16_t	Encoder_Angle_Buffer;			//编码器上一次值
	int32_t		Encode_Position;			//编码器位置数据，包含多圈计数
	int16_t 	Encoder_1MS_Speed;			//1ms编码器速度
	int16_t 	Speed_Filter;				//编码器速度滤波控制位
	uint32_t 	Encoder_Offest_Data;		        //编码器修正值
	uint8_t		Encoder_Direction;			//编码器方向，与q轴方向相同为0，相反为1
}Encoder;

extern  uint16_t Tx_Encoder[2];
extern  uint16_t Rx_Encoder[2];

//编码器变量
extern  Encoder encoder1;

void Start_Encoder_GET(Encoder *encoder);
void Encoder_Data_Deal(Encoder *encoder);

#endif

