
#include "control_loop.h"
#include "string.h"


//定义控制环全局结构体
_Control_Loop	Current_Q_Loop;	//q轴电流环
_Control_Loop	Current_D_Loop;	//d轴电流环
_Control_Loop	Speed_Loop;		//速度环
_Control_Loop	Position_Loop;	//位置环

_PID_Control	Current_Q_PID;	//q轴电流pid
_PID_Control	Current_D_PID;	//d轴电流pid
_PID_Control	Speed_PI;		//速度pi
_PID_Control	Position_P;		//位置p + 前馈	

uint8_t			Loop_Count;		//环路周期计数

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

//电流控制环路
void Current_Loop_Control(_Control_Loop *loop)
{
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
void Speed_Loop_Control(_Control_Loop *loop)
{

}

//位置控制环路
void Position_Loop_Control(_Control_Loop *loop)
{

}

//模拟模型
//电流环模型输出前馈电压
uint8_t Current_Loop_Model(_Forward *forward)
{

}
//速度环模型输出前馈电流
uint8_t Current_Loop_Model(_Forward *forward)
{

}
//位置环模型输出前馈速度
uint8_t Current_Loop_Model(_Forward *forward)
{

}

//环路参数初始化
void Control_Loop_Init(_Control_Loop *loop)
{
	loop->Word.ALL = 0;
	loop->Back = 0;
	loop->Input_Target = 0;
	loop->Output_Result = 0;
	memcpy(&loop->Loop_PID,0,sizeof(loop->Loop_PID));
	memcpy(&loop->Forward,0,sizeof(loop->Forward));
}






