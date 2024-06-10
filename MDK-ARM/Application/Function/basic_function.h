
#ifndef __BASIC_FUNCTION_H
#define __BASIC_FUNCTION_H

#include "stm32f1xx_hal.h"
#include "foc.h"

//编码器校正变量
typedef struct _Encoder_Offset
{
    uint16_t	Offest_Time_Basic;		//时基值	8kHz = 1:125us	16kHz = 1:62.5us	32kHz = 1:31.25us
    uint16_t	Number_Offest_Count;	//编码器累加实际次数
    uint32_t	Offest_Integral;		//编码器累加存储值
    uint16_t	Virtual_Angle;			//虚拟实际角度
    uint16_t	Offest_Table_Count;		//线性补偿计数
    uint8_t		Offest_Model;			//校正内置模式 1：零位校正 2：线性正向 3：线性反向
}Encoder_Offest;

typedef struct _Motor_Status
{
    uint8_t		Last_Work_Model;		//上一次的工作模式
    uint8_t		Last_PWM_Enable;		//上一次使能状态
    int32_t		Last_Encoder_Position;	//上一次编码器的位置
    int32_t		Last_1MS_Speed;			//上一次1ms编码器位置计算得速度
    
    uint8_t     Position_Ins;           //位置环位置来源
    uint8_t     Speed_Ins;              //速度环速度来源
    uint8_t     Current_Ins;            //电流环电流来源
}Motor_Status;

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
uint8_t Model_Control(_Control_Data *Data,_Control_Status *Status);
//PWM逻辑使能控制
void Enable_Logic_Control(void);

//1ms速度计算
void Speed_1MS(void);
//1ms中断回调函数
void Interrupt_1MS(void);

#endif

