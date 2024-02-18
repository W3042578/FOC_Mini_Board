
#include "usart.h"
#include "adc.h"
#include "tim.h"

#include "foc.h"
#include "parameter.h"
#include "control_loop.h"
#include "table.h"
#include "encoder.h"
#include "modbus.h"
#include "basic_function.h"


//功能层&底层交互
//主要存放出于功能层需要对底层修改的函数

//定义全局变量
uint16_t	Encoder_Offset_Delay;	//编码器初始角度对齐延迟
uint16_t	Number_Offest_Count;	//编码器累加实际次数
uint8_t		Last_Work_Model;		//上一次的工作模式
int32_t		Last_Encoder_Position;	//上一次编码器胡相对位置
int32_t		Last_1MS_Speed;			//上一次1ms编码器位置计算得速度
uint32_t 	ADC_Data[2];			//ADC采样DMA储存数据地址
uint8_t		Angle_Origin_End;	//编码器原点修正完成标志

//底层配置
//底层初始化配置
void STM32_Infrastructure_Init(void)
{
	//校准ADC采样
	//同步注入采样中需将adc模式设置为连续采样模式才可以使用，仍需对规则组进行部分配置，如规则采样触发，采样数，数据对齐方式
	//同步注入采样中adc1为主采样器，adc2为从配置器，因此触发adc1即可触发adc2
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);

	//开启规则组常规采样
	HAL_ADC_Start(&hadc2); 
	HAL_ADC_Start(&hadc1);

  	//开启注入组采样，注意注入采样控制中断的处理函数中会关闭注入中断使能位
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);

 	 //同步dual模式为多模式采样，同步规则采样需要开启DMA，注入采样建立在同步采样基础上
	HAL_ADCEx_MultiModeStart_DMA(&hadc1,ADC_Data,2);
	
	//开启cc4比较通道触发adc注入采样
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

	//定时器2开启1ms中断
	HAL_TIM_Base_Start_IT(&htim2);

	//开启串口DMA发送和接受
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)&Rx_Data,RX_BUFF_LONG);

	//使能串口空闲中断
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
}

//STM32 HAL 三相PWM比较值设置
void STM32_HAL_PWM_SET_Compare(FOC_Motor *motor)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->Ta);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->Tb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motor->Tc);
}


//编码器&角度
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

//编码器校准 获取编码器对应alpha轴零位修正角度值，判断编码器方向与q轴正方向是否一致
void Get_Initial_Angle_Offest(FOC_Motor *motor)
{
	//编码器零位校正
	if(Control_Word.Work_Model == 1 && Control_Word.PWM_Enable == 1)//判断工作模式1且进入PWM使能
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
			Angle_Origin_End = 1;	//编码器原点校正完成，准备进行线性度校正
			Control_Word.Work_Model = 0;		//校正完成退出校正模式并关闭PWM使能
			Control_Word.PWM_Enable = 0;
			Work_Status.bits.Angle_Offest = 0;	//编码器校正位清零 允许下一次进入零位校准
		}
		//编码器线性度校正
		if(Angle_Origin_End == 1)
		{
			
		}
	}
}


//电流采样
//获取两相电流采样修正值  包含偏置电压
void ADC_Current_Offest(FOC_Motor *motor)
{
	uint32_t	Add_ADC_Offect_U,Add_ADC_Offect_V;
	uint16_t	Number_ADC_Offect;
	Work_Status.bits.Offest_Current = 1;
	Add_ADC_Offect_U = Add_ADC_Offect_V = 0;
	Number_ADC_Offect = 32;
	motor->Ia_Offect = 0;				//校准值置零，避免采样回调函数中修正值影响直接采得的数据
	motor->Ib_Offect = 0;

	HAL_Delay(1000);
	//平均减小误差
	for(uint8_t i = 1;i<=Number_ADC_Offect;i++)
	{
		//累加考虑到电流获取负号，负负得正
		Add_ADC_Offect_U = Add_ADC_Offect_U - motor->Ia;
		Add_ADC_Offect_V = Add_ADC_Offect_V - motor->Ib;
		HAL_Delay(2);//延迟2ms 62.5us获取一次电流采样值，确保2ms内采样值至少更新一次
	}
	motor->Ia_Offect = Add_ADC_Offect_U >> 5;
	motor->Ib_Offect = Add_ADC_Offect_V >> 5;
	motor->Ia = motor->Ib =0;	
}



//应用层功能
//工作模式控制
void Model_Control(FOC_Motor *motor)
{
	//模式切换判断
	if(Last_Work_Model != Control_Word.Work_Model)
	{
		if(Control_Word.PWM_Enable == 1)//PWM使能输出情况下不允许更改工作模式
			Control_Word.Work_Model = Last_Work_Model;
		else							//PWM非使能允许模式切换，但需要清零各PID器中的积分量
		{
			Current_Q_PID.Integral_Sum = 0;
			Current_D_PID.Integral_Sum = 0;
			Speed_PI.Integral_Sum = 0;
			Position_PI.Integral_Sum = 0;
		}
	}
		
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
				Angle_Origin_End = 0;		//重置原点修正指示位
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
				//目标速度 单位：dec(编码器单个数值单位) 360度对应65536dec
				Position_PI.Feedback = Control_Loop.Target_Position;
				//PI计算
				PID_Control_Deal(&Position_PI);
				//输出控制电流
				Control_Loop.Target_Speed = Position_PI.Output_Sum;
			}
		

		//速度环模式：PID控制速度闭环输出
		case 5:
			//快速对2取余，速度环频率为电流环一半
			if((Control_Loop.Loop_Count & 0x02) == 0)
			{
				//速度回馈
				Speed_PI.Feedback = Encoder1.Encoder_1MS_Speed * 1.0923; //1.0923 = 65536/60000 1rpm转1dec/ms
				//目标速度 单位：rpm
				Speed_PI.Feedback = Control_Loop.Target_Speed;
				//PI计算
				PID_Control_Deal(&Speed_PI);
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
			PID_Control_Deal(&Current_Q_PID);
			PID_Control_Deal(&Current_D_PID);
			//输出控制电压
			//考虑前馈解耦 电流PI输出电流 再由电流计算输出电压 需要获取电机Lq、Ld
			motor->Uq = Current_Q_PID.Output_Sum;
			motor->Ud = Current_D_PID.Output_Sum;
		break;
			
		//默认0模式，不做输出
		default:
			// Error_Message.bits.Control_Loop_Error = 1;
		break;
	}
	//保存上一次控制模式
	Last_Work_Model = Control_Word.Work_Model;
	//环路计数累加
	Control_Loop.Loop_Count ++;
	//环路计数到达限制清零
	if(Control_Loop.Loop_Count > 8)
	{
		Control_Loop.Loop_Count = 0;
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

//电机控制死区补偿
void Dead_Time_Compensate(FOC_Motor *motor)
{
	//根据回馈电流极性判断SVPWM生成三相占空比正负时间补偿
	
}

//1ms速度计算
void Speed_1MS(void)
{
	//判断是否首次进入1ms中断
	if(Work_Status.bits.Interrupt_1MS_Init == 0)
	{
		Work_Status.bits.Interrupt_1MS_Init = 1;
		//令过去位置对于当前位置，避免起步错误速度
		Last_Encoder_Position = Encoder1.Encode_Position;
	}
	//1ms速度等于 当前位置 - 1ms前位置
	Encoder1.Encoder_1MS_Speed = Encoder1.Encode_Position - Last_Encoder_Position;
	//编码器速度范围限制,超出限制以上一次速度代替
	if((Encoder1.Encoder_1MS_Speed > 16384) || (Encoder1.Encoder_1MS_Speed < -16384))
		Encoder1.Encoder_1MS_Speed = Last_1MS_Speed;
	//更新过去1ms时间编码器位置、1ms编码器速度
	Last_Encoder_Position = Encoder1.Encode_Position;
	Last_1MS_Speed = Encoder1.Encoder_1MS_Speed;
	//1ms速度计算1个电流环周期机械角速度并乘2
	//乘2因为当前周期获取得的速度为上一个周期发送指令获取得，在下一周期生效
	Motor1.Speed_Angle = Encoder1.Encoder_1MS_Speed >> 3;
}

//1ms中断回调函数 外部输入输出数据、温度保护、电机状态保护、速度计算处理
void Interrupt_1MS(void)
{
	//考虑到编码器给出为绝对值位置信号，速度的求解需要结合时间，将编码器速度计算放在定时器中
	Speed_1MS();
	//更新PID控制参数数据
	PID_Control_Update();
}


