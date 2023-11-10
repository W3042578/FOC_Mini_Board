#include "control_loop.h"
#include "math.h"
#include "main.h"
#include "foc.h"

//PID调参输出只针对具体Ud,Uq，无需考虑矢量算法部分
//PID参数初始化
//电流环PID输出为Ud，Uq且最大为16384*Udc
void Current_PID_Init(FOC_PID *foc_pid)
{
	foc_pid->Kp = 10;
	foc_pid->Ki = 10;
	foc_pid->Kd = 10;
	
	foc_pid->Error = 0;
	foc_pid->Last_Error = 0;
	
	foc_pid->Error_Low_Limit = -Motor1.Udc *200;
	foc_pid->Error_Upper_Limit = Motor1.Udc * 200;		//误差上限  9459 = 16384/sqrt(3)
	foc_pid->Integral_Low_Limit = -5000;
	foc_pid->Integral_Upper_Limit = 5000;
	foc_pid->Different_Low_Limit = -5000;
	foc_pid->Different_Upper_Limit = 5000;
	foc_pid->Output_Low_Limit = -Motor1.Udc *500;
	foc_pid->Output_Upper_Limit = Motor1.Udc * 500;
	
	Loop_Time_Count = 0;
}
void Speed_PID_Init(FOC_PID *foc_pid)
{
	foc_pid->Kp = 10;
	foc_pid->Ki = 10;
	foc_pid->Kd = 10;
	
	foc_pid->Error = 0;
	foc_pid->Last_Error = 0;
	 
	foc_pid->Error_Low_Limit = -600;
	foc_pid->Error_Upper_Limit = 600;
	foc_pid->Integral_Low_Limit = -2000;
	foc_pid->Integral_Upper_Limit = 2000;
	foc_pid->Different_Low_Limit = -1000;
	foc_pid->Different_Upper_Limit = 1000;
	foc_pid->Output_Low_Limit = -2048;
	foc_pid->Output_Upper_Limit = 2048;
}
void Position_PID_Init(FOC_PID *foc_pid)
{
	foc_pid->Kp = 4;
	foc_pid->Ki = 0;
	foc_pid->Kd = 5;
	
	foc_pid->Error = 0;
	foc_pid->Last_Error = 0;
	
	foc_pid->Error_Low_Limit = -8192;
	foc_pid->Error_Upper_Limit = 8192;
	foc_pid->Integral_Low_Limit = 0;
	foc_pid->Integral_Upper_Limit = 0;
	foc_pid->Different_Low_Limit = -1000;
	foc_pid->Different_Upper_Limit = 1000;
	foc_pid->Output_Low_Limit = -600;
	foc_pid->Output_Upper_Limit = 600;
}


//PID控制
//参考https://blog.csdn.net/qq_38833931/article/details/80630960
void PID_Control(FOC_PID* foc_pid)
{
	foc_pid->Error = foc_pid->Expect - foc_pid->Feed_Back;
	foc_pid->Integral = foc_pid->Integral + foc_pid->Error;
	foc_pid->Different = foc_pid->Error - foc_pid->Last_Error;
	
	//误差限幅
	if(foc_pid->Error>foc_pid->Error_Upper_Limit)
		foc_pid->Error = foc_pid->Error_Upper_Limit;
	if(foc_pid->Error<foc_pid->Error_Low_Limit)
		foc_pid->Error = foc_pid->Error_Low_Limit;
	//积分限幅
	if(foc_pid->Integral>foc_pid->Integral_Upper_Limit)
		foc_pid->Integral = foc_pid->Integral_Upper_Limit;
	if(foc_pid->Integral<foc_pid->Integral_Low_Limit)
		foc_pid->Integral =foc_pid->Integral_Low_Limit;
	//差分限幅
	if(foc_pid->Different>foc_pid->Different_Upper_Limit)
		foc_pid->Different = foc_pid->Different_Upper_Limit;
	if(foc_pid->Different<foc_pid->Different_Low_Limit)
		foc_pid->Different = foc_pid->Different_Low_Limit;
	//PID输出
	foc_pid->Output = foc_pid->Kp*foc_pid->Error + foc_pid->Ki * foc_pid->Integral + foc_pid->Kd * foc_pid->Different;
	
	//输出限幅  积分快速反馈  限饱和
	if(foc_pid->Output>foc_pid->Output_Upper_Limit)
		{
			foc_pid->Output = foc_pid->Output_Upper_Limit;
			foc_pid->Integral = foc_pid->Output_Upper_Limit - foc_pid->Kp * foc_pid->Error - foc_pid->Kp * foc_pid->Kd * foc_pid->Different;
		}
	if(foc_pid->Output<foc_pid->Output_Low_Limit)
		{
			foc_pid->Output = foc_pid->Output_Low_Limit;
			foc_pid->Integral = foc_pid->Output_Low_Limit + foc_pid->Kp * foc_pid->Error + foc_pid->Kp * foc_pid->Kd * foc_pid->Different;

		}
	//当前误差为下一次的过去误差
	foc_pid->Last_Error = foc_pid->Error;
}



