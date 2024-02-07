#include "control_loop.h"

#include "main.h"
#include "foc.h"


//定义控制环全局结构体
_Control_Loop	Control_Loop;
_PID_Control	Current_Q_PID,Current_D_PID,Speed_PI,Position_PI;	

//并联式PID控制  
//参考https://blog.csdn.net/qq_38833931/article/details/80630960
//实际只用PI 并联式抗积分饱和
void PID_Control_Deal(_PID_Control * PID_Control)
{
	int32_t Integral_Data;	//积分结果变量
	int32_t	Init_Output;	//未限幅前总输出
	int32_t Anti_Wind;	//抗饱和结果变量
	
	PID_Control->Error = PID_Control->Expect - PID_Control->Feedback;
	PID_Control->Integral_Sum = PID_Control->Integral_Sum + PID_Control->Error;

	PID_Control->Proportion_Sum = PID_Control->Proportion * PID_Control->Error;   //Kp * Error
	Integral_Data = PID_Control->Integral * PID_Control->Integral_Sum;		//Ki * Integral
	PID_Control->Difference_Sum = PID_Control->Difference * (PID_Control->Error - PID_Control->Last_Error); //Kd * Difference
	//比例限幅
	if(PID_Control->Proportion_Sum > PID_Control->Proportion_Limit)
		PID_Control->Proportion_Sum = PID_Control->Proportion_Limit;
	else if(PID_Control->Proportion_Sum < -PID_Control->Proportion_Limit)
		PID_Control->Proportion_Sum = -PID_Control->Proportion_Limit;
	
	//积分限幅
	if(Integral_Data > PID_Control->Integral_Limit)
		Integral_Data = PID_Control->Integral_Limit;
	else if(Integral_Data < -PID_Control->Integral_Limit)
		Integral_Data = PID_Control->Integral_Limit;
	//微分限幅
	if(PID_Control->Difference_Sum > PID_Control->Difference_Limit)
		PID_Control->Difference_Sum = PID_Control->Difference_Limit;
	else if(PID_Control->Difference_Sum < -PID_Control->Difference_Limit)
		PID_Control->Difference_Sum = -PID_Control->Difference_Limit;
	
	//总输出限幅并计算抗饱和
	PID_Control->Output_Sum = PID_Control->Proportion_Sum + Integral_Data + PID_Control->Difference_Sum;
	Init_Output = PID_Control->Output_Sum;
	if(PID_Control->Output_Sum > PID_Control->Output_limit)
		PID_Control->Output_Sum = PID_Control->Output_limit;
	else if(PID_Control->Output_Sum < -PID_Control->Output_limit)
		PID_Control->Output_Sum = -PID_Control->Output_limit;
	Anti_Wind = PID_Control->Antiback * (Init_Output - PID_Control->Output_Sum);
	
	//积分有限幅 控制抗饱和反馈避免积分量正负间跳变
	if(((Anti_Wind > 0) && (Anti_Wind < PID_Control->Integral_Sum)) || ((Anti_Wind < 0) && (Anti_Wind > PID_Control->Integral_Sum)))
		PID_Control->Integral_Sum = PID_Control->Integral_Sum - Anti_Wind;
	else
		PID_Control->Integral_Sum = 0;
}

//PID结构体参数初始化
void PID_Control_Init(_PID_Control *PID)
{
	PID->Proportion = 0;
	PID->Integral = 0;
	PID->Difference = 0;
	PID->Antiback = 0;
	
	PID->Error = 0;
	PID->Last_Error = 0;
	PID->Feedback = 0;
	PID->Expect = 0;
	PID->Feedforward = 0;
	
	PID->Proportion_Sum = 0;
	PID->Integral_Sum = 0;
	PID->Difference_Sum = 0;
	PID->Output_Sum = 0;
	
	PID->Proportion_Limit = 0;
	PID->Integral_Limit = 0;
	PID->Difference_Limit = 0;
}

//PID控制环具体参数初始化
void Control_Loop_Init(_Control_Loop *Loop)
{
	Loop->Target_Q_Current = 0;
	Loop->Target_D_Current = 0;
	Loop->Target_Speed = 0;
	Loop->Target_Position = 0;
	Loop->Loop_Count = 0;
	
	Loop->Current_Q_Proportion = 50;
	Loop->Current_Q_Integral = 1;
	Loop->Current_Q_Difference = 0;
	
	Loop->Current_D_Proportion = 50;
	Loop->Current_D_Integral = 1;
	Loop->Current_D_Difference = 0;
	
	Loop->Speed_Proportion = 25;
	Loop->Speed_Integral = 1;

	Loop->Position_Proportion = 10;
	Loop->Position_Integral = 0;
	Loop->Position_Feedforward = 1;
	
	Loop->Current_Q_Output_Limit = 1.5 * 2048;
	Loop->Current_D_Output_Limit = 1 * 2048;
	Loop->Speed_Output_Limit = 0;
	Loop->Position_Output_Limit = 0;
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




