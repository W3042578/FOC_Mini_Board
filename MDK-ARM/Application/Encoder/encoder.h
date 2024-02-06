
#ifndef __ENCODER_H
#define __ENCODER_H
#include "foc.h"
#include "stm32f1xx_hal.h"


typedef struct _Encoder
{
	uint8_t		Type;							//编码器类型
	uint8_t		Multi_Bit;						//多圈位数
	uint8_t		Single_Bit;						//单圈位数
	uint16_t	Encoder_Angle;					//编码器16位值
	uint16_t	Encoder_Angle_Buffer;			//编码器上一次值
	int32_t		Encode_Position;				//编码器位置数据，包含多圈计数
	int16_t 	Encoder_Speed_Angle;			//1ms编码器速度
	int16_t 	Encoder_Speed_Angle_Buffer;		//上一次1ms编码器速度
	int16_t 	Encoder_Speed_Angle_Loop;		//速度环编码器速度
	int16_t 	Encoder_Speed_Angle_Loop_Buffer;//上一次速度环编码器速度
	int16_t 	Speed_Filter;					//速度滤波控制位
	int16_t		Last_Encoder_Loop;				//速度环内部计算速度用
	int16_t 	Speed_Filter_Loop;              //速度环速度计算滤波
	uint32_t 	Encoder_Offest_Data;		        	//编码器修正值
	uint8_t		Encoder_Direction;				//编码器方向，与q轴方向相同为0，相反为1
	uint16_t 	Encoder_Direction_Position;		//编码器方向判断用位置
}Encoder;

extern  uint16_t Tx_Encoder[2];
extern  uint16_t Rx_Encoder[2];

//编码器变量
extern  Encoder encoder1;

void Start_Encoder_GET(Encoder *encoder);
void Encoder_Data_Deal(Encoder *encoder);

#endif

