
#ifndef __BASIC_FUNCTION_H
#define __BASIC_FUNCTION_H

#include "stm32f1xx_hal.h"
#include "foc.h"
#include "parameter.h"

//底层配置
//底层初始化配置
void STM32_Infrastructure_Init(void);
//STM32 HAL 三相PWM比较值设置
void STM32_HAL_PWM_SET_Compare(_FOC_Motor *motor);

//电流采样
//获取两相电流采样修正值
void ADC_Current_Offest(_FOC_Motor *motor);


//编码器
//角度转换
void Encoder_To_Electri_Angle(_FOC_Motor *motor);


//应用层功能
//工作模式控制
enum Motor_Work_Model	//工作模式控制
{
	NULL_MODEL = 0,		//该环不工作
	OPEN_LOOP,
	CURRENT_LOOP,
	SPEED_LOOP,
	POSITION_LOOP,
	SENSELESS,
	NORMAL_CONTROL,
	OPEN_VOLTAGE,
	DUTY_CONTROL,
	EOCODER_OFFEST,
	MTPA_CONTROL,
	DEBUG_ENGINE
};
enum Offest_Model
{
	OFFEST_INIT	= 1,		//初始角校正
	POSITIVE_OFFEST,		//线性正向
	NEGITIVE_OFFEST			//线性反向
};
uint8_t Model_Control(_Control_Data *Data,_Control_Status *Status);
//PWM逻辑使能控制
void Enable_Logic_Control(void);

//1ms速度计算
void Speed_1MS(void);
//1ms中断回调函数
void Interrupt_1MS(void);

#endif

