
#ifndef __BASIC_FUNCTION_H
#define __BASIC_FUNCTION_H



#include "stm32f1xx_hal.h"
#include "foc.h"


//功能层&底层交互
//主要存放出于功能层需要对底层修改的函数

//同步采样用电流采样变量
extern uint32_t ADC_Data[2];

//底层配置
//底层初始化配置
void STM32_Infrastructure_Init(void);
//STM32 HAL 三相PWM比较值设置
void STM32_HAL_PWM_SET_Compare(FOC_Motor *motor);

//电流采样
//获取两相电流采样修正值
void ADC_Current_Offest(FOC_Motor *motor);


//编码器
//角度转换
void Encoder_To_Electri_Angle(FOC_Motor *motor);
//初始修正
void Get_Initial_Angle_Offest(FOC_Motor *motor);



//应用层功能
//工作模式控制
void Model_Control(FOC_Motor *motor);
//PWM逻辑使能控制
void Enable_Logic_Control(void);

//应用算法
//最大转矩比控制MTPA
void MTPA_Control(FOC_Motor *motor);
//电流前馈解耦
void Current_Forward_Feedback(FOC_Motor *motor);
//死区补偿
void Dead_Time_Compensate(FOC_Motor *motor);

//1ms速度计算
void Speed_1MS(void);
//1ms中断回调函数
void Interrupt_1MS(void);

#endif

