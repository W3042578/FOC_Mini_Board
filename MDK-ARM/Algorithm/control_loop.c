#include "control_loop.h"

#include "main.h"
#include "foc.h"


//定义控制环全局结构体
_Control_Loop	Control_Loop;
_PID_Control	Current_Q_PID;
_PID_Control	Current_D_PID;
_PID_Control	Speed_PI;
_PID_Control	Position_PI;	

//并联式PID控制  
//参考https://blog.csdn.net/qq_38833931/article/details/80630960
//实际只用PI 并联式抗积分饱和
void PID_Control_Deal(_PID_Control * pid_control)
{
	int32_t Integral_Data;	//积分结果变量
	int32_t	Init_Output;	//未限幅前总输出
	int32_t Anti_Wind;	//抗饱和结果变量
	
	pid_control->Error = pid_control->Expect - pid_control->Feedback;
	pid_control->Integral_Sum = pid_control->Integral_Sum + pid_control->Error;

	pid_control->Proportion_Sum = pid_control->Proportion * pid_control->Error;   //Kp * Error
	Integral_Data = pid_control->Integral * pid_control->Integral_Sum;		//Ki * Integral
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
	
	//总输出限幅并计算抗饱和
	pid_control->Output_Sum = pid_control->Proportion_Sum + Integral_Data + pid_control->Difference_Sum;
	Init_Output = pid_control->Output_Sum;
	if(pid_control->Output_Sum > pid_control->Output_limit)
		pid_control->Output_Sum = pid_control->Output_limit;
	else if(pid_control->Output_Sum < -pid_control->Output_limit)
		pid_control->Output_Sum = -pid_control->Output_limit;
	Anti_Wind = pid_control->Antiback * (Init_Output - pid_control->Output_Sum);
	
	//积分有限幅 控制抗饱和反馈避免积分量正负间跳变
	if(((Anti_Wind > 0) && (Anti_Wind < pid_control->Integral_Sum)) || ((Anti_Wind < 0) && (Anti_Wind > pid_control->Integral_Sum)))
		pid_control->Integral_Sum = pid_control->Integral_Sum - Anti_Wind;
	else
		pid_control->Integral_Sum = 0;
}

//PID结构体参数初始化
void PID_Control_Init(_PID_Control *pid)
{
	pid->Proportion = 0;
	pid->Integral = 0;
	pid->Difference = 0;
	pid->Antiback = 0;
	
	pid->Error = 0;
	pid->Last_Error = 0;
	pid->Feedback = 0;
	pid->Expect = 0;
	pid->Feedforward = 0;
	
	pid->Proportion_Sum = 0;
	pid->Integral_Sum = 0;
	pid->Difference_Sum = 0;
	pid->Output_Sum = 0;
	
	pid->Proportion_Limit = 0;
	pid->Integral_Limit = 0;
	pid->Difference_Limit = 0;
}

//PID控制环具体参数初始化
void Control_Loop_Init(_Control_Loop *loop)
{
	loop->Target_Q_Current = 0;
	loop->Target_D_Current = 0;
	loop->Target_Speed = 0;
	loop->Target_Position = 0;
	loop->Loop_Count = 0;
	
	loop->Current_Q_Proportion = 50;
	loop->Current_Q_Integral = 1;
	loop->Current_Q_Difference = 0;
	
	loop->Current_D_Proportion = 50;
	loop->Current_D_Integral = 1;
	loop->Current_D_Difference = 0;
	
	loop->Speed_Proportion = 25;
	loop->Speed_Integral = 1;

	loop->Position_Proportion = 10;
	loop->Position_Integral = 0;
	loop->Position_Feedforward = 1;
	
	loop->Current_Q_Output_Limit = 1.5 * 2048;
	loop->Current_D_Output_Limit = 1 * 2048;
	loop->Speed_Output_Limit = 0;
	loop->Position_Output_Limit = 0;
}

//1ms中断更新PID结构体数据
void PID_Control_Update(void)
{
	//电流Iq PID
	Current_Q_PID.Proportion = Control_Loop.Current_Q_Proportion;
	Current_Q_PID.Integral = Control_Loop.Current_Q_Integral;
	Current_Q_PID.Difference = Control_Loop.Current_Q_Difference;
	Current_Q_PID.Integral_Limit = Control_Loop.Current_Q_Output_Limit;
	Current_Q_PID.Proportion_Limit = Control_Loop.Current_Q_Output_Limit;
	Current_Q_PID.Difference_Limit = Control_Loop.Current_Q_Output_Limit>>1;
	//电流Id PID
	Current_D_PID.Proportion = Control_Loop.Current_D_Proportion;
	Current_D_PID.Integral = Control_Loop.Current_D_Integral;
	Current_D_PID.Difference = Control_Loop.Current_D_Difference;
	Current_D_PID.Integral_Limit = Control_Loop.Current_D_Output_Limit;
	Current_D_PID.Proportion_Limit = Control_Loop.Current_D_Output_Limit;
	Current_D_PID.Difference_Limit = Control_Loop.Current_D_Output_Limit>>1;
	
	//速度环 PI
	Speed_PI.Proportion = Control_Loop.Speed_Proportion;
	Speed_PI.Integral = Control_Loop.Speed_Integral;
	Speed_PI.Integral_Limit = Control_Loop.Speed_Output_Limit;
	Speed_PI.Proportion_Limit = Control_Loop.Speed_Output_Limit;
	
	//位置环 PI+前馈
	Position_PI.Proportion = Control_Loop.Position_Proportion;
	Position_PI.Integral = Control_Loop.Position_Integral;
	Position_PI.Integral_Limit = Control_Loop.Position_Output_Limit;
	Position_PI.Proportion_Limit = Control_Loop.Position_Output_Limit;
	Position_PI.Feedforward = Control_Loop.Position_Feedforward;
}




