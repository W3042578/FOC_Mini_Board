
#include "main.h"
#include "math.h"
#include "tim.h"
#include "foc.h"
#include "parameter.h"
#include "control_loop.h"
#include "sin_cos.h"
#include "encoder.h"
#include "basic_function.h"


//功能层&底层交互
//主要存放出于功能层需要对底层修改的函数

//定义全局变量
uint16_t	Encoder_Offset_Delay;	//编码器初始角度对齐延迟
uint16_t	Number_Offest_Count;	//编码器累加实际次数

//获取编码器角度并转换为电角度
void Encoder_To_Electri_Angle(FOC_Motor *motor)
{
	uint16_t electri_angle;
	uint32_t offest_angle;
	
	//角度获取与电流采样同周期
	Encoder_Data_Deal(&Encoder1); //获取编码器角度，速度，位置
	Start_Encoder_GET(&Encoder1); //开启编码器DMA，获取下一次使用角度
	
	
	//电机原点位置对齐alpha轴
	if(Encoder1.Encoder_Angle < motor->Initial_Angle_Offset)
			offest_angle = Encoder1.Encoder_Angle - motor->Initial_Angle_Offset + 65535;
	else
			offest_angle = Encoder1.Encoder_Angle - motor->Initial_Angle_Offset;
	
	//电机速度补偿计算用角度
	if(motor->Speed_Angle > 0)
	{
		offest_angle = offest_angle + motor->Speed_Angle;
		if(offest_angle > 65535)
			offest_angle = offest_angle - 65535;
	}
	else
	{
		if(offest_angle < motor->Speed_Angle)
			offest_angle = offest_angle + 65535 - motor->Speed_Angle;
		else
			offest_angle = offest_angle - motor->Speed_Angle;
	}
	//使用与运算快速取余 t % 2`(n) 等价于 t & (2`(n) - 1)
	//参考https://blog.csdn.net/lonyw/article/details/80519652
	electri_angle = (motor->Polar * offest_angle) & 0xFFFE;
	//查表获取电角度对应三角函数值
	motor->Sin_Angle = SIN_COS_TABLE[(electri_angle >> 7)];
	motor->Cos_Angle = SIN_COS_TABLE[((electri_angle >> 7)+128) & 0x1ff];
}

//STM32 HAL 三相PWM比较值设置
void STM32_HAL_PWM_SET_Compare(FOC_Motor *motor)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->Ta);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->Tb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motor->Tc);
}

//编码器校准 
//获取编码器对应alpha轴零位修正角度值
//正反转编码器获取线性化查表补偿数据并判断编码器方向与q轴正方向是否一致
//获取零位置角度修正值
void Get_Initial_Angle_Offest(FOC_Motor *motor)
{
	//编码器零位校正
	if(Control_Word.Work_Model == 1 && Work_Status.bits.Angle_Offest != 0)//判断工作模式1且为零位校准状态进入校准
	{
		if(Encoder_Offset_Delay > 0)	//Encoder_Offset_Delay非零减到零
			Encoder_Offset_Delay --;
		else	//Encoder_Offset_Delay为零后进行编码器累加计数工作，累加指定次数
		{
			motor->Initial_Angle_Offset = motor->Initial_Angle_Offset + Encoder1.Encoder_Angle;
			Number_Offest_Count --;
		}
		if(Number_Offest_Count == 0)//指定次数累加后平均获得零位校准值
		{
			motor->Initial_Angle_Offset = motor->Initial_Angle_Offset >> (Control_Word.Number_Angle_Offest);
			Control_Word.Work_Model = 0;			//校正完成退出校正模式并关闭PWM使能
			Control_Word.PWM_Enable = 0;
			Work_Status.bits.Angle_Offest = 0;//编码器校正位清零 允许下一次进入零位校准
		}
	}
}

//获取两相电流采样修正值  包含偏置电压
void ADC_Current_Offest(FOC_Motor *motor)
{
	uint32_t Add_ADC_Offect_U,Add_ADC_Offect_V;
	uint16_t	Number_ADC_Offect;
	Work_Status.bits.Offest_Current = 1;
	Add_ADC_Offect_U = Add_ADC_Offect_V = 0;
	Number_ADC_Offect = 32;
	motor->Ia_Offect = 0;				//校准值置零，避免采样回调函数中修正值影响直接采得的数据
	motor->Ib_Offect = 0;
	
	motor->Ualph = 0;//Ualph = 0V
	motor->Ubeta = 0;
	SVPWM(motor);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->Ta);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->Tb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motor->Tc);	
	
	//使能drv8313
	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);	
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	HAL_Delay(1000);
	//平均减小误差
	for(uint8_t i = 1;i<=Number_ADC_Offect;i++)
	{
		Add_ADC_Offect_U = Add_ADC_Offect_U - motor->Ia;
		Add_ADC_Offect_V = Add_ADC_Offect_V - motor->Ib;//此处累加使用负号考虑到校准时获取的数值是负数
		HAL_Delay(2);
	}
	motor->Ia_Offect = Add_ADC_Offect_U >> 5;
	motor->Ib_Offect = Add_ADC_Offect_V >> 5;
	motor->Ia = motor->Ib =0;	
}

//电机控制死区补偿
void Dead_Time_Compensate(FOC_Motor *motor)
{
	//根据回馈电流极性判断SVPWM生成三相占空比正负时间补偿
	
}

//工作模式控制
//函数工作在FOC电流环中
void Model_Control(FOC_Motor *motor)
{
	//根据控制字判断工作环
	switch(Control_Word.Work_Model)
	{	
		//校正模式：初始角度校正和编码器线性补偿
		case 1:
			motor->Ualph = 2048 * Control_Word.Angle_Initial_Voltage;//Ualph = 3V
			motor->Ubeta = 0;
			if(Work_Status.bits.Angle_Offest == 0)
			{
				Work_Status.bits.Angle_Offest = 1;
				motor->Initial_Angle_Offset = 0;
				Encoder_Offset_Delay = 32;
				Number_Offest_Count = 1<<(Control_Word.Number_Angle_Offest);
			}
		break;
		
		//占空比模式：按照设置输出指定单相满额占空比（考虑采样最大98%）
		case 2:
			//对输入占空比数值作限制
			if(Control_Word.Duty_Model_A > 96)
			{
				Control_Word.Duty_Model_A = 96;
			}
			if(Control_Word.Duty_Model_B > 96)
			{
				Control_Word.Duty_Model_B = 96;
			}
			if(Control_Word.Duty_Model_C > 96)
			{
				Control_Word.Duty_Model_C = 96;
			}
		break;
		
		//电压开环模式：按照设置的Uq、Ud电压开环控制
		case 3:
			motor->Ud = 0;
			motor->Uq = Control_Word.Open_Loop_Voltage * 2048;
		break;
		
		//速度环模拟无感控制：模拟速度增加控制速度开环输出
		case 7:
			
		break;
		
		//位置环模式：PID控制相对位置闭环输出
		case 6:
			//快速对4取余，速度环频率为电流环0.25倍
			if((Control_Loop.Loop_Count & 0x04) == 0)
			{
				//相对位置回馈
				Position_PI.Feedback = Encoder1.Encode_Position;
				//目标速度 单位：
				Position_PI.Feedback = Control_Loop.Target_Position;
				//PI计算
				PID_Control_Deal(Position_PI);
				//输出控制电流
				Control_Loop.Target_Q_Current = Position_PI.Output_Sum;
			}
		

		//速度环模式：PID控制速度闭环输出
		case 5:
			//快速对2取余，速度环频率为电流环一半
			if((Control_Loop.Loop_Count & 0x02) == 0)
			{
				//速度回馈
				Speed_PI.Feedback = motor->Iq;
				//目标速度 单位：
				Speed_PI.Feedback = Control_Loop.Target_Speed;
				//PI计算
				PID_Control_Deal(Speed_PI);
				//输出控制电流
				Control_Loop.Target_Q_Current = Speed_PI.Output_Sum;
			}
			
		//电流环模式：PID控制电流Iq、Id闭环输出
		case 4:
			//输入反馈电流
			Current_Q_PID.Feedback = motor->Iq;
			Current_D_PID.Feedback = motor->Id;
			//输入期望电流
			Current_Q_PID.Expect = Control_Loop.Target_Q_Current;
			Current_D_PID.Expect = Control_Loop.Target_D_Current;
			//PID计算
			PID_Control_Deal(Current_Q_PID);
			PID_Control_Deal(Current_D_PID);
			//输出控制电压
			motor->Uq = Current_Q_PID.Output_Sum;
			motor->Ud = Current_D_PID.Output_Sum;

			//环路计数累加
			Control_Loop.Loop_Count ++;
			if(Control_Loop.Loop_Count > 4)
				Control_Loop.Loop_Count = 0;
		break;
			
		//默认0模式，不做输出
		default:
//			Error_Message.bits.Control_Loop_Error = 1;
		break;
	}
}

//PWM使能控制
void Enable_Logic_Control(void)
{
	//紧急停止触发、停止PWM输出并关闭驱动模块工作
	if(Control_Word.Energency_Stop == 1)
	{
		//关闭驱动模块
		HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);
		//关闭三相PWM输出
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
		//PWM使能状态置0
		Work_Status.bits.Enable_Status = 0;
	}
	else
	{
		if(Control_Word.PWM_Enable == 1)
		{
			//开启三相PWM输出
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
			//使能驱动模块
			HAL_GPIO_WritePin(Power_Reset_GPIO_Port,Power_Reset_Pin,GPIO_PIN_SET);
			//PWM使能状态置1
			Work_Status.bits.Enable_Status = 1;
		}
		else
		{
			//关闭驱动模块
			HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);
			//关闭三相PWM输出
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			//PWM使能状态置0
			Work_Status.bits.Enable_Status = 0;
		}
	}
}

//1ms中断回调函数 外部输入输出数据、温度保护、电机状态保护处理
void Interrupt_1MS(void)
{
	//更新PID控制参数数据
	PID_Control_Update();
}


