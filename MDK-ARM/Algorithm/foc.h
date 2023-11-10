#ifndef __FOC_H
#define __FOC_H

#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "spi.h"
#include "usart_control.h" 

//数学运算 x16 = x65536
//#define _sqrt3_x16		113511    
//#define _sqrt3_3_x16	37837
//#define _2sqrt3_3_x16	75674
//#define _sqrt3_2_x16	56755
//#define _2pi_x16			411774
//#define	_2pi_360_x16	1144




typedef struct _FOC_MOTOR
{
	uint8_t Polar;								//极对数
	uint16_t Ts;									//拟合SVPWM电压参考矢量周期
	uint16_t Ts_Limit;
	
	int8_t Udc;									//母线电压
	int32_t Ud,Uq;
	int32_t Ualph,Ubeta;
	int16_t Ia,Ib;
	int16_t Ia_Offect,Ib_Offect;
	int16_t Ialph,Ibeta;
	int16_t Id,Iq;
	uint32_t Umax;							//考虑pwm输出无法实现100%的限制电压，为最大电压93.75% = 15/16
	uint32_t Uref;							//Ualph和Ubeta直接给出的合成电压平方值
	int32_t U1,U2,U3,m32;						//三相线上电压状态
	uint8_t Sa,Sb,Sc;
	int32_t Tx,Ty;
	uint8_t Sector_Add,Sector_Actual;	//扇区
	uint16_t temp16;						//计算Tx,Ty和不超过Ts
	
	uint16_t Encoder;						//编码器编码值
	int32_t Encoder_Angle;
	int16_t Encoder_Speed_Angle;			//1ms时间计算速度
	int16_t Encoder_Speed_Angle_Loop;		//速度环内部计算速度，与速度环同周期
	int16_t Speed_Filter;
	int16_t Last_Encoder;					//1ms时间计算速度用
	int16_t Last_Encoder_Loop;				//速度环内部计算速度用
	int16_t Speed_Filter_Loop;
	int8_t	Encoder_Tag;				//编码器磁有效标志
	uint16_t Encoder_Offest;		//编码器修正值
	
	uint32_t Mechanical_Angle;	//机械角度
	uint32_t Elecrical_Angle;		//电气角度
	int16_t Sin_Angle;					//电角度sin、cos值
	int16_t Cos_Angle;
	int32_t Ta,Tb,Tc;						//三相分配上桥打开时间（1：上桥打开，下桥关闭，0：下桥打开，上桥关闭）
	
	

	
}FOC_Motor;

//电机结构体变量
extern FOC_Motor Motor1;
extern uint16_t Tx_Encoder[2];
extern uint16_t Rx_Encoder[2];

void FOC_Motor_Init(FOC_Motor *motor);
uint16_t Get_Angle_MT6813(FOC_Motor *motor);
void Encoder_Offest(FOC_Motor *motor);
void ADC_Current_Offest(FOC_Motor *motor);
void Get_Electrical_Angle(FOC_Motor *motor);

void Clark_Transform(FOC_Motor *motor);
void Park_Transform(FOC_Motor *motor);
void Inverse_Park_Transform(FOC_Motor *motor);
void SVPWM(FOC_Motor *motor);
void FOC_Control(FOC_Motor *motor);


void Set_Motor_Channel_Time(FOC_Motor *motor);

#endif
