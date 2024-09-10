
#include "string.h"
#include "control_loop.h"
#include "parameter.h"
#include "foc.h"
#include "basic_function.h"
#include "encoder.h"
#include <math.h>
#include <stdlib.h>

//定义控制环全局结构体
_Control_Loop	Open_Voltage_Data;	//电压开环
_Control_Loop	Current_Loop_Data;	//q轴电流环
_Control_Loop	Speed_Loop_Data;	//速度环
_Control_Loop	Position_Loop_Data;	//位置环
_PID_Control	Current_Q_PID;		//q轴电流pid
_PID_Control	Current_D_PID;		//d轴电流pid
_PID_Control	Speed_PI;			//速度pi
_PID_Control	Position_P;			//位置p + 前馈	

uint8_t 	polar;
int32_t 	differ_buffer;
//编码器校准模式
enum Encoder_Offest_Model
{
	ZERO_OFFEST = 0,
	TURN_CHECK,
	END_Check
};
//PID 限制初始化
static void PID_Init(_PID_Control *pid_control)
{
	pid_control->Proportion_Limit = 20000;
	pid_control->Integral_Limit = 20000;
	pid_control->Difference_Limit = 2000;
	pid_control->Output_limit = 30000;
}
//PID 积分量清除
static void PID_Reset_Time(_PID_Control *pid_control)
{
	pid_control->Integral_Data = 0;
	pid_control->Last_Error = 0;
}
//位置式抗饱和并联PID
static void Parallel_PID(_PID_Control *pid_control)
{
	pid_control->Error = pid_control->Expect - pid_control->Feedback;	
	pid_control->Proportion_Sum = pid_control->Proportion * pid_control->Error;  	//Kp * Error
	pid_control->Integral_Sum = pid_control->Integral * pid_control->Integral_Data;				//Ki * Integral
	pid_control->Difference_Sum = pid_control->Difference * (pid_control->Error - pid_control->Last_Error); //Kd * Difference
	pid_control->Last_Error = pid_control->Error;
	
	//避免积分进入饱和区，快速反应
	if((pid_control->Error < 0) && (pid_control->Integral_Sum > -(pid_control->Integral_Limit)))
	{
		pid_control->Integral_Data += pid_control->Error;	
	}
	else if((pid_control->Error > 0) && (pid_control->Integral_Sum < pid_control->Integral_Limit))
	{
		pid_control->Integral_Data += pid_control->Error;
	}
	
	//比例限幅
	if(pid_control->Proportion_Sum > pid_control->Proportion_Limit)
		pid_control->Proportion_Sum = pid_control->Proportion_Limit;
	else if(pid_control->Proportion_Sum < -pid_control->Proportion_Limit)
		pid_control->Proportion_Sum = -pid_control->Proportion_Limit;
	//积分限幅
	if(pid_control->Integral_Sum > pid_control->Integral_Limit)
		pid_control->Integral_Sum = pid_control->Integral_Limit;
	else if(pid_control->Integral_Sum < -pid_control->Integral_Limit)
		pid_control->Integral_Sum = -pid_control->Integral_Limit;
	//微分限幅
	if(pid_control->Difference_Sum > pid_control->Difference_Limit)
		pid_control->Difference_Sum = pid_control->Difference_Limit;
	else if(pid_control->Difference_Sum < -pid_control->Difference_Limit)
		pid_control->Difference_Sum = -pid_control->Difference_Limit;
	
	//总输出限幅
	pid_control->Output_Sum = pid_control->Proportion_Sum + pid_control->Integral_Sum + pid_control->Difference_Sum;
	if(pid_control->Output_Sum > pid_control->Output_limit)
	{
		pid_control->Output_Sum = pid_control->Output_limit;
	}
	else if(pid_control->Output_Sum < -pid_control->Output_limit)
	{
		pid_control->Output_Sum = -pid_control->Output_limit;
	}

}

//开环控制
void Open_Voltage_Loop(uint8_t *source,_Control_Loop *loop)
{
	//静态
	static 	uint16_t 	Pulse_Buffer = 0;
	static 	int32_t 	count = 0;
	static 	uint8_t 	polar = 0;
	static	uint8_t		state = 0;	//状态字 BIT0:完成初始化 BIT1:滞留完成初始化

	//局部
	_Control_Data 	*data = &Control_Data;
	_Control_Status *status = &Control_Status;
	_FOC_Motor 		*motor = &Motor1;
	_Encoder 		*encoder = &Encoder1;
	int32_t 		differ_buffer;
	
	if(data->Control_Word.bits.PWM_Enable | _TEST(&state,BIT1))
	{
		switch (*source)
		{
			case OPEN_VOLTAGE:	//开环电压
				if(data->Open_Loop_Voltage > (SQRT3_3 * motor->Udc))
					motor->Uq = SQRT3_3 * (motor->Udc << INIT_SCALE);
				else
					motor->Uq = data->Open_Loop_Voltage << INIT_SCALE;
				motor->Ud = 0;
			break;
			case DUTY_CONTROL:	//占空比
				if(data->Duty_Data.Phase_A > 96)
					data->Duty_Data.Phase_A = 96;
				if(data->Duty_Data.Phase_B > 96)
					data->Duty_Data.Phase_B = 96;
				if(data->Duty_Data.Phase_C > 96)
					data->Duty_Data.Phase_C = 96;
			break;
			case EOCODER_OFFEST://编码器校准
				_SET(&state,BIT1);		//滞留初始化置位
				status->Work_Status.bits.Offest_Encoder = 1;		//编码器校正状态置位
				motor->Ud = data->Encoder_Offest.Angle_Initial_Voltage << INIT_SCALE;	//Ud电压强拖
				motor->Uq = 0;
				if(data->Encoder_Offest.Offest_Wait > 0)			//等待时间计数
				{
					data->Encoder_Offest.Offest_Wait --;
					if(data->Control_Word.bits.PWM_Enable == 0)
					{
						data->Encoder_Offest.Offest_Wait = 0;
						status->Error_status.bits.PWM_Enable = 1;		//使能关闭错误
						data->Encoder_Offest.Offest_Model = END_Check;	//报错进入结束模式
					}
				}
				else
				{
					switch (data->Encoder_Offest.Offest_Model)
					{
						case ZERO_OFFEST:
							if(_TEST(&state,BIT0) == 0)		//待初始化
							{	//校正模式初始化
								motor->Initial_Offset = 0;				//清零零位校正角
								motor->Elecrical_Angle = 0;
								motor->Sin_Angle = 0;							//0电角度对应三角函数
								motor->Cos_Angle = 4096;	
								data->Encoder_Offest.Number_Offest_Count = 0;	//位置数据获取计数清零			
								data->Encoder_Offest.Offest_Integral = 0;		//清零编码器累加值				
								data->Encoder_Offest.Offest_Wait = 2000;		//零位校正记录编码器数值间隔时间
								_SET(&state,BIT0);		//初始化置位
							}
							else if(data->Encoder_Offest.Number_Offest_Count < (1 << data->Encoder_Offest.Number_Angle_Offest))
							{
								data->Encoder_Offest.Offest_Integral += encoder->Encoder_Pulse;	//位置数据累加
								data->Encoder_Offest.Offest_Wait = 400;			//累加间隔时间
								data->Encoder_Offest.Number_Offest_Count ++;		//计数累加
							}
							else
							{	//累加位置平均得出电机对齐坐标位置偏差值
								motor->Initial_Offset = data->Encoder_Offest.Offest_Integral >> data->Encoder_Offest.Number_Angle_Offest;							
								data->Encoder_Offest.Offest_Model = TURN_CHECK;			//进入转动模式
								data->Encoder_Offest.Offest_Wait = 500;				//子模式过渡时间	1/16k * 8k = 0.5s	
								_CLEAN(&state,BIT0);									//初始化位清除					
							}
						break;
						case TURN_CHECK:
							if(_TEST(&state,BIT0) == 0)	//初始化
							{
								//机械角补偿过原点偏差 加上编码器静态偏差避免第一次进入就产生位置跳变
								Pulse_Buffer = motor->Mechanical_Angle + encoder->Encoder_Deviation;	
								data->Encoder_Offest.Offest_Wait = 10;		//虚拟电角度累加间隔时间							
								data->Encoder_Offest.Error_Time = 0;			//错误计数清零
								_SET(&state,BIT0);
							}
							else
							{
								data->Encoder_Offest.Offest_Wait = 2;				//虚拟电角度累加间隔时间
								if((data->Encoder_Offest.Virtual_Angle & 0x3ff) == 0)	//虚拟电角度累加到 对16384取余
								{
									differ_buffer = data->Encoder_Offest.Differ_Check;
									data->Encoder_Offest.Differ_Check = motor->Mechanical_Angle - Pulse_Buffer;	
									Pulse_Buffer = motor->Mechanical_Angle;
									differ_buffer = abs(differ_buffer - data->Encoder_Offest.Differ_Check);
//									if((abs(data->Encoder_Offest.Differ_Check) > (1 << (encoder->Single_Bit - 1))) \
//										&& (polar >= 1))	//编码器位置单圈返回起点 至少一对极	
									if(0)
									{
										motor->Polar = polar;	//更新极对数
										polar = 0;
										if(count < 0)			//更新编码器方向
										{
											_NEGA(&encoder->Encoder_Status,BIT0);	//取反
											motor->Initial_Offset = 65535 - motor->Initial_Offset;
										}						
										data->Encoder_Offest.Offest_Model = END_Check;		//进入结束
									}
//									else if (((data->Encoder_Offest.Virtual_Angle & 0x7fff) == 0) && (data->Encoder_Offest.Virtual_Angle != 0))	//对32768取余为零且自身非零
//									{
//										polar ++;																			
//										count += data->Encoder_Offest.Differ_Check;
//									}
//									else if(differ_buffer > (1 << (encoder->Single_Bit/2)))	//实际数据获取错误 单次角度变化超过单圈位数一半
//									{
//										data->Encoder_Offest.Error_Time ++;	//错误累计
//										if(data->Encoder_Offest.Error_Time > 5) //报错
//										{											
//											status->Error_status.bits.Encoder_Offset = 1;	//置位编码器校正错误
//											data->Encoder_Offest.Offest_Model = END_Check;	//报错进入结束模式
//										}
//									}
								}
								data->Encoder_Offest.Virtual_Angle += data->Encoder_Offest.Virtual_Increment;//虚拟电角度累加								
							}
						break;
						case END_Check:
							motor->Ud = 0;
							motor->Uq = 0;		
							polar = 0;				
							data->Encoder_Offest.Error_Time = 0;						
							data->Encoder_Offest.Number_Offest_Count = 0;
							data->Encoder_Offest.Offest_Integral = 0;
							data->Encoder_Offest.Virtual_Angle = 0;
							data->Encoder_Offest.Offest_Model = ZERO_OFFEST;		//初始化校正状态
							data->Control_Word.bits.Sub_Work_Model_Buffer = 0;		//校准模式结束，置空模式 			
							status->Work_Status.bits.Offest_Encoder = 0;			//编码器校正状态置位 结束
							_CLEAN(&state,BIT0|BIT1);
						break;
						default:

						break;
					}
				}
			break;

			default:	//默认电压由电流环控制
				
			break;
		}
	}
}
//电流环
void Current_Loop(uint8_t *source,_Control_Loop *loop)
{
	//局部
	_Control_Data 	*data = &Control_Data;
	_Control_Status *status = &Control_Status;
	_FOC_Motor 		*motor = &Motor1;
	_FOC_Driver		*driver = &Driver1;

	//获取反馈电流Iq,Id
	Clark_Transform(motor);
	//换算获取Iq,Id
	Park_Transform(motor);

	if(!data->Control_Word.bits.PWM_Enable)	//关闭使能清除PID积分、微分累积
	{
		PID_Reset_Time(&Current_Q_PID);
		PID_Reset_Time(&Current_D_PID);
	}
	else
	{
		switch (*source)
		{
			case DIRECT_CURRENT:
				Current_Q_PID.Feedback = motor->Iq;
				Current_D_PID.Feedback = motor->Id;
//				Current_Q_PID.Expect = data->Command_Iq * driver->Scale_1_10;	//电流 * 比例值转adc数值
				Current_D_PID.Expect = 0;
				Parallel_PID(&Current_Q_PID);
				Parallel_PID(&Current_D_PID);
				motor->Uq = Current_Q_PID.Output_Sum;
				motor->Ud = Current_D_PID.Output_Sum;
			break;
			case MTPA_CONTROL:
			
			break;
			case FIELD_WEAK:
			
			break;
			default:

			break;
		}
	}		
}

//速度环
void Speed_Loop(uint8_t *source,_Control_Loop *loop)
{

}

//位置环
void Position_Loop(uint8_t *source,_Control_Loop *loop)
{
	//获取电角度 约补偿20us到达第一个pwm中电 62.5+20us到达第二个pwm中电
	Encoder_To_Electri_Angle(&Motor1);
}

//模拟模型
//电流环模型输出前馈电压
uint8_t Current_Loop_Model(_Forward *forward)
{

	return 0;
}
//速度环模型输出前馈电流
uint8_t Speed_Loop_Data_Model(_Forward *forward)
{

	return 0;
}
//位置环模型输出前馈速度
uint8_t Position_Loop_Data_Model(_Forward *forward)
{

	return 0;
}

//环路启动初始化
void Loop_Init(void)
{
	PID_Init(&Current_Q_PID);
	PID_Init(&Current_D_PID);
	PID_Init(&Speed_PI);
	PID_Init(&Position_P);
}





