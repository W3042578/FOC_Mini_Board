#ifndef __FOC_H
#define __FOC_H

#include "stm32f1xx_hal.h"

typedef struct _FOC_MOTOR
{
//电机参数
	uint8_t	 	Polar;			//极对数
	uint32_t	Ld,Lq;			//d和q轴电感量
	uint32_t	Flux_Linkage;	//永磁体磁链 单位：Wb

	int8_t 		Udc;			//母线电压
	int32_t 	Ud,Uq;
	int32_t 	Ualph,Ubeta;
	int16_t 	Ia,Ib;
	int16_t 	Ia_Offect,Ib_Offect;
	int16_t 	Ialph,Ibeta;
	int16_t 	Id,Iq;
	uint32_t 	Umax;			//pwm输出无法实现100%占空比
//SVPWM
	uint16_t 	Ts;				//拟合SVPWM电压参考矢量周期
	uint32_t 	Uref;			//Ualph和Ubeta直接给出的合成电压平方值
	int32_t 	U1,U2,U3,m32;	//三相线上电压状态
	uint8_t 	Sa,Sb,Sc;
	int32_t 	Tx,Ty;			
	uint8_t 	Sector;			//扇区
		
	uint16_t	Mechanical_Angle;	//机械角度
	uint16_t 	Elecrical_Angle;	//电气角度
	uint32_t 	Initial_Angle_Offset;	//零位修正角，对齐alpha轴
	int32_t		Speed_Angle;		//机械转速
	int32_t		Elecrical_Speed;	//电气转速We 单位rad/s
	int16_t 	Sin_Angle;			//电角度sin、cos值
	int16_t 	Cos_Angle;
	int32_t 	Ta,Tb,Tc;			//三相分配上桥打开时间（1：上桥打开，下桥关闭，0：下桥打开，上桥关闭）
	uint16_t	Td;			//死区时间对应计数值
	
	uint8_t 	Direction;			//电机工作方向
	uint8_t		Offest_Direction;	//电机修正方向
}FOC_Motor;

//电机结构体变量
extern FOC_Motor Motor1;


void Clark_Transform(FOC_Motor *motor);
void Park_Transform(FOC_Motor *motor);
void Inverse_Park_Transform(FOC_Motor *motor);
void SVPWM(FOC_Motor *motor);
void PWM_Time_Count(FOC_Motor *motor);
void FOC_Control(FOC_Motor *motor);



#endif
