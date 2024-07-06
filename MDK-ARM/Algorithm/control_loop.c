
#include "string.h"
#include "control_loop.h"
#include "parameter.h"
#include "foc.h"
#include "basic_function.h"
#include "encoder.h"
#include <math.h>
#include <stdlib.h>

//定义控制环全局结构体
_Control_Loop	Open_Voltage_Loop;	//电压开环
_Control_Loop	Current_Q_Loop;		//q轴电流环
_Control_Loop	Current_D_Loop;		//d轴电流环
_Control_Loop	Speed_Loop;			//速度环
_Control_Loop	Position_Loop;		//位置环
_PID_Control	Current_Q_PID;		//q轴电流pid
_PID_Control	Current_D_PID;		//d轴电流pid
_PID_Control	Speed_PI;			//速度pi
_PID_Control	Position_P;			//位置p + 前馈	
uint8_t			Loop_Count;			//环路周期计数

//编码器校准模式
enum Encoder_Offest_Model
{
	ZERO_OFFEST = 0,
	TURN_CHECK,
	END_Check
};

//静态全局
static uint16_t Pulse_Buffer;
static uint8_t polar;

//抗饱和并联PID
void Parallel_PID(_PID_Control *pid_control)
{
	int32_t Integral_Data;	//积分结果变量

	pid_control->Error = pid_control->Expect - pid_control->Feedback;
	
	pid_control->Proportion_Sum = pid_control->Proportion * pid_control->Error;  	//Kp * Error
	Integral_Data = pid_control->Integral * pid_control->Integral_Sum;				//Ki * Integral
	pid_control->Difference_Sum = pid_control->Difference * (pid_control->Error - pid_control->Last_Error); //Kd * Difference
	//比例限幅
	if(pid_control->Proportion_Sum > pid_control->Proportion_Limit)
		pid_control->Proportion_Sum = pid_control->Proportion_Limit;
	else if(pid_control->Proportion_Sum < -pid_control->Proportion_Limit)
		pid_control->Proportion_Sum = -pid_control->Proportion_Limit;
	//积分限幅
	if(Integral_Data > pid_control->Integral_Limit)
		Integral_Data = pid_control->Integral_Limit;
	else if(Integral_Data < -pid_control->Integral_Limit)
		Integral_Data = pid_control->Integral_Limit;
	//微分限幅
	if(pid_control->Difference_Sum > pid_control->Difference_Limit)
		pid_control->Difference_Sum = pid_control->Difference_Limit;
	else if(pid_control->Difference_Sum < -pid_control->Difference_Limit)
		pid_control->Difference_Sum = -pid_control->Difference_Limit;
	
	//总输出限幅
	pid_control->Output_Sum = pid_control->Proportion_Sum + Integral_Data + pid_control->Difference_Sum;
	if(pid_control->Output_Sum > pid_control->Output_limit)
	{
		pid_control->Output_Sum = pid_control->Output_limit;
		if(pid_control->Error < 0)//避免积分进入饱和区，快速反应
			pid_control->Integral_Sum = pid_control->Integral_Sum + pid_control->Error;
	}
	else if(pid_control->Output_Sum < -pid_control->Output_limit)
	{
		pid_control->Output_Sum = -pid_control->Output_limit;
		if(pid_control->Error > 0)
			pid_control->Integral_Sum = pid_control->Integral_Sum + pid_control->Error;
	}
}

//开环控制
void Open_Voltage_Control(uint8_t *source,_Control_Loop *loop)
{
	_Control_Data * data = &Control_Data;
	_Control_Status *status = &Control_Status;
	_FOC_Motor * motor = &Motor1;
	_Encoder *encoder = &Encoder1;

	int32_t buffer;
	uint32_t test;
	if(data->Control_Word.bits.PWM_Enable == 1)
	switch (*source)
	{
		case OPEN_VOLTAGE:	//开环电压
			if(data->Open_Loop_Voltage > (SQRT3_3 * motor->Udc))
				motor->Uq = SQRT3_3 * (motor->Udc << _INIT_SCALE);
			else
				motor->Uq = data->Open_Loop_Voltage << _INIT_SCALE;
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
			status->Work_Status.bits.Offest_Encoder = 1;		//编码器校正状态置位
			motor->Ud = data->Encoder_Offest.Angle_Initial_Voltage << _INIT_SCALE;	//Ud电压强拖
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
						if(status->Work_Status.bits.Angle_Offest == 0)		//初始化
						{	//校正模式初始化
							motor->Initial_Angle_Offset = 0;				//清零零位校正角
							motor->Elecrical_Angle = 0;
							motor->Sin_Angle = 0;							//0电角度对应三角函数
							motor->Cos_Angle = 4096;	
							data->Encoder_Offest.Number_Offest_Count = 0;	//位置数据获取计数清零			
							data->Encoder_Offest.Offest_Integral = 0;		//清零编码器累加值				
							data->Encoder_Offest.Offest_Wait = 4000;		//零位校正记录编码器数值间隔时间
							status->Work_Status.bits.Angle_Offest = 1;		//初始化置位
						}
						else if(data->Encoder_Offest.Number_Offest_Count < (1 << data->Encoder_Offest.Number_Angle_Offest))
						{
							data->Encoder_Offest.Offest_Integral += encoder->Encoder_Pulse;	//位置数据累加
							data->Encoder_Offest.Offest_Wait = 800;			//累加间隔时间
							data->Encoder_Offest.Number_Offest_Count ++;		//计数累加
						}
						else
						{	//累加位置平均得出电机对齐坐标位置偏差值
							motor->Initial_Angle_Offset = data->Encoder_Offest.Offest_Integral >> data->Encoder_Offest.Number_Angle_Offest;
							data->Encoder_Offest.Offest_Integral = 0;
							status->Work_Status.bits.Angle_Offest = 0;				//初始化位清除
							data->Encoder_Offest.Offest_Model = TURN_CHECK;			//进入转动模式
							data->Encoder_Offest.Offest_Wait = 1000;				//子模式过渡时间	1/16k * 8k = 0.5s						
						}
					break;
					case TURN_CHECK:
						if(status->Work_Status.bits.Angle_Offest == 0)	//初始化
						{
							Pulse_Buffer = encoder->Encoder_Pulse;
							data->Encoder_Offest.Offest_Wait = 1000;		//虚拟电角度累加间隔时间
							status->Work_Status.bits.Angle_Offest = 1;
						}
						else
						{
							data->Encoder_Offest.Offest_Wait = 250;				//虚拟电角度累加间隔时间
							if((data->Encoder_Offest.Virtual_Angle & 0x3ff) == 0)	//虚拟电角度累加到 对16384取余
							{
								buffer = data->Encoder_Offest.Differ_Check;
								data->Encoder_Offest.Differ_Check = encoder->Encoder_Pulse - Pulse_Buffer;
								Pulse_Buffer = encoder->Encoder_Pulse;
								if(data->Encoder_Offest.Differ_Check < 0)
									test = -data->Encoder_Offest.Differ_Check;
								else
									test = data->Encoder_Offest.Differ_Check;
								if(test >= 32768)	//编码器位置大跳变 判断为单圈返回原点
								{
									motor->Polar = polar;			//根据虚拟角度累加结果判断极对数
									data->Encoder_Offest.Offest_Model = END_Check;			//进入结束模式
								}
								else if (((data->Encoder_Offest.Virtual_Angle & 0x7fff) == 0) && (data->Encoder_Offest.Virtual_Angle != 0))	//对32768取余非零且自身非零
								{
									polar ++;
								}
								else if((data->Encoder_Offest.Differ_Check > (10 * buffer)) \
								|| ((10 *  data->Encoder_Offest.Differ_Check) < buffer))	//实际数据获取错误
								{
									data->Encoder_Offest.Error_Time ++;	//错误累计
									if(data->Encoder_Offest.Error_Time > 5) //报错
									{
										data->Encoder_Offest.Error_Time = 0;
										status->Error_status.bits.Encoder_Offset = 1;	//置位编码器校正错误
										data->Encoder_Offest.Offest_Model = END_Check;	//报错进入结束模式
									}
								}
							}
							data->Encoder_Offest.Virtual_Angle += 256;		//虚拟电角度累加
							motor->Elecrical_Angle = data->Encoder_Offest.Virtual_Angle;	//虚拟电角度更新给实际电角度执行
						}
					break;
					case END_Check:
						motor->Ud = 0;
						motor->Uq = 0;
						data->Encoder_Offest.Virtual_Angle = 0;
						data->Control_Word.bits.Sub_Work_Model_Buffer = 0;		//校准模式结束，置空模式 
						data->Encoder_Offest.Offest_Model = ZERO_OFFEST;		//初始化校正状态	
						status->Work_Status.bits.Offest_Encoder = 0;			//编码器校正状态置位 结束
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
//电流控制环路
void Current_Loop_Control(uint8_t *source,_Control_Loop *loop)
{
	//获取反馈电流Iq,Id
	Clark_Transform(&Motor1);
	//换算获取Iq,Id
	Park_Transform(&Motor1);

	loop->Loop_PID.Expect = loop->Input_Target;
	loop->Loop_PID.Feedback = loop->Back;
	Parallel_PID(&loop->Loop_PID);

	switch (loop->Word.bits.Loop_Control)
	{
		case PID_FILTER:
		
		break;
		case FORWARD_CONTROL:
		
		break;
		case FORWARD_FILTER:
		
		break;
		default:
		loop->Output_Result = loop->Loop_PID.Output_Sum;
		break;
	}	
}

//速度控制环路
void Speed_Loop_Control(uint8_t *source,_Control_Loop *loop)
{

}

//位置控制环路
void Position_Loop_Control(uint8_t *source,_Control_Loop *loop)
{
	//获取电角度
	Encoder_To_Electri_Angle(&Motor1);
}

//模拟模型
//电流环模型输出前馈电压
uint8_t Current_Loop_Model(_Forward *forward)
{

	return 0;
}
//速度环模型输出前馈电流
uint8_t Speed_Loop_Model(_Forward *forward)
{

	return 0;
}
//位置环模型输出前馈速度
uint8_t Position_Loop_Model(_Forward *forward)
{

	return 0;
}

//环路参数初始化
void Control_Loop_Init(_Control_Loop *loop)
{
	loop->Word.ALL = 0;
	loop->Back = 0;
	loop->Input_Target = 0;
	loop->Output_Result = 0;
	memset(&loop->Loop_PID,0,sizeof(loop->Loop_PID));
	memset(&loop->Forward,0,sizeof(loop->Forward));
}

//环路参数更新
void Control_Loop_Update(_Control_Loop *loop)
{
	//参数更新函数：模式切换判断
	if(1)
	{
		Current_Q_PID.Integral_Sum = 0;
		Current_D_PID.Integral_Sum = 0;
		Speed_PI.Integral_Sum = 0;
		Position_P.Integral_Sum = 0;
	}
}





