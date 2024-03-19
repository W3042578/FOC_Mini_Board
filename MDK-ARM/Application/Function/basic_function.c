
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
//前一时刻状态变量保存
uint8_t		Last_Work_Model;		//上一次的工作模式
uint8_t		Last_PWM_Enable;		//上一次使能状态
int32_t		Last_Encoder_Position;	//上一次编码器的位置
int32_t		Last_1MS_Speed;			//上一次1ms编码器位置计算得速度

//电流采样
uint32_t 	ADC_Data[2];			//ADC采样DMA储存数据地址

//编码器校准
uint16_t	Offest_Time_Basic;		//时基值	8kHz = 1:125us	16kHz = 1:62.5us	32kHz = 1:31.25us
uint16_t	Number_Offest_Count;	//编码器累加实际次数
uint32_t	Offest_Integral;		//编码器累加存储值
uint16_t	Virtual_Angle;			//虚拟实际角度
uint16_t	Offest_Table_Count;		//线性补偿计数
uint8_t		Offest_Model;			//校正内置模式 1：零位校正 2：线性正向 3：线性反向

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
	
	//使能驱动模块
	HAL_GPIO_WritePin(Power_Reset_GPIO_Port,Power_Reset_Pin,GPIO_PIN_SET);
	
}

//STM32 HAL 三相PWM比较值设置
void STM32_HAL_PWM_SET_Compare(FOC_Motor *motor)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->Ta);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->Tb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motor->Tc);
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


//编码器&角度
//获取编码器角度并转换为电角度
void Encoder_To_Electri_Angle(FOC_Motor *motor)
{
	int32_t offest_angle;
	

	//角度获取与电流采样同周期
	Encoder_Data_Deal(&Encoder1); //获取编码器角度，速度，位置
	Start_Encoder_GET(&Encoder1); //开启编码器DMA，获取下一次使用角度
	
	//电机原点位置对齐alpha轴
	if(Encoder1.Encoder_Angle < motor->Initial_Angle_Offset)
		motor->Mechanical_Angle = Encoder1.Encoder_Angle - motor->Initial_Angle_Offset + 65535;
	else
		motor->Mechanical_Angle = Encoder1.Encoder_Angle - motor->Initial_Angle_Offset;
	
	//电机速度补偿计算用角度
	offest_angle = motor->Mechanical_Angle + motor->Speed_Angle;
	if(offest_angle > 65535)
	{
		offest_angle = offest_angle - 65535;
	}
	else if(offest_angle < 0)
	{
		offest_angle = offest_angle + 65535;
	}
	
	//使用与运算快速取余 t % 2`(n) 等价于 t & (2`(n) - 1)
	//参考https://blog.csdn.net/lonyw/article/details/80519652
	motor->Elecrical_Angle = (motor->Polar * offest_angle) & 0xFFFE;
	//查表获取电角度对应三角函数值
	motor->Sin_Angle = SIN_COS_TABLE[(motor->Elecrical_Angle >> 7)];
	motor->Cos_Angle = SIN_COS_TABLE[((motor->Elecrical_Angle >> 7)+128) & 0x1ff];
}

//编码器校准 获取编码器对应alpha轴零位修正角度值，判断编码器方向与q轴正方向是否一致
void Get_Initial_Angle_Offest(FOC_Motor *motor)
{
	int16_t Offest_Differen;	//编码器实际值与虚拟值偏差
	//编码器零位校正
	if(Control_Word.bits.Work_Model == 1 && Control_Word.bits.PWM_Enable == 1)//判断工作模式1且进入PWM使能
	{
		if(Offest_Time_Basic > 0)	//时基延时计数
			Offest_Time_Basic --;	//时基值	8kHz = 1:125us	16kHz = 1:62.5us	32kHz = 1:31.25us
		else	//时基值到达
		{
			//校正内置模式判断
			//校正模式中包含三种模式：1）初始角校正 2）线性正向 3）线性反向
			switch (Offest_Model)
			{
				case 1:		//零位校正记录
					//零位校正值累加
					Offest_Integral = Offest_Integral + Encoder1.Encoder_Angle;
					//零位每次记录时基值 320*62.5us = 20ms
					Offest_Time_Basic = 320;
					//累加次数结束
					Number_Offest_Count ++;
					//根据累加次数判断是否结束零位校正
					if(Number_Offest_Count > (1 << Control_Data.Number_Angle_Offest))
					{
						//更新零位修正值
						motor->Initial_Angle_Offset = Offest_Integral >> (Control_Data.Number_Angle_Offest);
						//累加次数清零
						Number_Offest_Count = 0;
						//进入线性正向
						Offest_Model = 2;
						//清零线性校正数组计数
						Offest_Table_Count = 0;
						//清零线性校正虚拟角
						Virtual_Angle = 0;
						//清零数组序号
						Offest_Table_Count = 0;
					}
					break;

				case 2:		//线性正向记录
					//虚拟角度对256快速取余
					if((Virtual_Angle & 0x00ff) == 0)
					{
						//获取编码器修正零位后数值
						Offest_Differen = motor->Mechanical_Angle - Virtual_Angle;
						//偏差值过大判断为电机正方向与编码器方向相反
						if(Offest_Differen > 256 || Offest_Differen < -256)
							motor->Offest_Direction = 1;
						//正向校正时记录偏差	
						Encoder_Line_Offest_Table[Offest_Table_Count] = Offest_Differen;
						//记录数组序号增加
						Offest_Table_Count ++;
					}
					//8*62.5us = 500us
					Offest_Time_Basic = 8;
					//判断虚拟角度值是否允许增加 ，避免溢出回零
					if(Virtual_Angle < 65535)
					{
						//虚拟角度值增加
						Virtual_Angle ++;
					}
					else if(Virtual_Angle == 65535)
					{
						//进入线性反向
						Offest_Model = 3;
						//进入模式3前等待时间 320 * 62.5us = 20ms
						Offest_Time_Basic = 320;
						//数组序号回退一步从255开始
						Offest_Table_Count --;
					}
					break;

				case 3:		//线性反向记录
					//虚拟角度对256快速取余
					if((Virtual_Angle & 0x00ff) == 0)
					{
						//获取编码器修正零位后数值
						Offest_Differen = motor->Mechanical_Angle - Virtual_Angle;
						//获取编码器修正零位后数值
						Offest_Differen = motor->Mechanical_Angle - Virtual_Angle;
						//偏差值过大判断为电机正方向与编码器方向相反
						if(Offest_Differen > 256 || Offest_Differen < -256)
							motor->Offest_Direction = 1;
						//正向校正时记录偏差	
						Encoder_Line_Offest_Table[Offest_Table_Count] = (Offest_Differen + Encoder_Line_Offest_Table[Offest_Table_Count])>>1;
						if(Offest_Table_Count > 0)
						//记录数组序号增加
						Offest_Table_Count --;
					}
					//8*62.5us = 500us
					Offest_Time_Basic = 8;
					//判断虚拟角度值是否允许减少 ，避免溢出回零
					if(Virtual_Angle > 0)
					{
						//虚拟角度值增加
						Virtual_Angle --;
					}
					else if(Virtual_Angle == 0)	//校正完成
					{
						//初始化校正模式
						Offest_Model = 1;
						//清除控制模式
						Control_Word.bits.Work_Model = 0;
						//退出校正模式并关闭PWM使能		
						Control_Word.bits.PWM_Enable = 0;
						//编码器校正位清零 允许下一次进入零位校准
						Work_Status.bits.Angle_Offest = 0;
						//清零Ud Uq
						motor->Ud = 0;
						motor->Uq = 0;
					}
					break;

				default: 	//校正模式错误
					Error_Message.bits.Encoder_Offset = 1;
					break;
			}
		}
	}
}


//应用层功能
//工作模式控制
void Model_Control(FOC_Motor *motor)
{
	uint16_t virtual_eletri_angle;	//虚拟电角度,编码器线性度校正用
		
	//模式切换判断
	if(Last_Work_Model != Control_Word.bits.Work_Model)
	{
		if(Control_Word.bits.PWM_Enable == 1)//PWM使能输出情况下不允许更改工作模式
			Control_Word.bits.Work_Model = Last_Work_Model;
		else							//PWM非使能允许模式切换，但需要清零各PID器中的积分量
		{
			Current_Q_PID.Integral_Sum = 0;
			Current_D_PID.Integral_Sum = 0;
			Speed_PI.Integral_Sum = 0;
			Position_PI.Integral_Sum = 0;
		}
	}
	
	//根据控制字判断工作环
	switch(Control_Word.bits.Work_Model)
	{	
		//校正模式：初始角度校正和线性补偿
		//校正模式中包含三种模式：1）初始角校正 2）线性正向 3）线性反向
		case 1:
			motor->Ud = 2048 * Control_Data.Angle_Initial_Voltage;//Ualph = 3V
			motor->Uq = 0;
			//判断进入校正起始
			if(Work_Status.bits.Angle_Offest == 0)
			{
				//校正模式仅起始进入一次
				Work_Status.bits.Angle_Offest = 1;
				//校正内置模式从零位校正开始
				Offest_Model = 1;	
				//清零零位校正角
				motor->Initial_Angle_Offset = 0;
				//清零编码器累加值
				Offest_Integral = 0;
				//零位第一次记录时基值 200ms 3200*62.5us 设置大些让电机拖动稳定后再记录
				Offest_Time_Basic = 3200;	
			}

			//校正内置模式判断
			//零位校正
			if(Offest_Model == 1)	
			{	
				//设置零位对应静态坐标电压
				motor->Sin_Angle = 0;
				motor->Cos_Angle = 4096;
			}
			//线性校正
			else if ((Offest_Model == 2) || (Offest_Model == 3))
			{
				//计算虚拟机械角转虚拟电角度
				virtual_eletri_angle = (motor->Polar * Virtual_Angle) & 0xFFFE;
				//查表获取电角度对应三角函数值
				motor->Sin_Angle = SIN_COS_TABLE[(virtual_eletri_angle >> 7)];
				motor->Cos_Angle = SIN_COS_TABLE[((virtual_eletri_angle >> 7)+128) & 0x1ff];
			}
		break;
		
		//占空比模式：按照设置输出指定单相满额占空比（考虑采样最大98%）
		case 2:
			//对输入占空比数值作限制
			if(Control_Data.Duty_Model_A > 96)
			{
				Control_Data.Duty_Model_A = 96;
			}
			if(Control_Data.Duty_Model_B > 96)
			{
				Control_Data.Duty_Model_B = 96;
			}
			if(Control_Data.Duty_Model_C > 96)
			{
				Control_Data.Duty_Model_C = 96;
			}
		break;
		
		//电压开环模式：按照设置的Uq、Ud电压开环控制
		case 3:
			motor->Ud = 0;
			motor->Uq = Control_Data.Open_Loop_Voltage * 2048;
		break;
		
		//速度环模拟无感控制：模拟速度增加控制速度开环输出
		//锁相环估算速度
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
			//判断是否进行MTPA控制且控制模式为电流环
			if((Control_Word.bits.MTPA == 1) && (Control_Word.bits.Work_Model == 4))
				MTPA_Control(motor);
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
			motor->Uq = Current_Q_PID.Output_Sum;
			motor->Ud = Current_D_PID.Output_Sum;
			
			if(Control_Word.bits.Current_Forward == 1)//判断是否进行电流前馈解耦
				Current_Forward_Feedback(motor);
		break;
		
		//默认0模式，不做输出
		default:
			// Error_Message.bits.Control_Loop_Error = 1;
		break;
	}
	//保存上一次控制模式
	Last_Work_Model = Control_Word.bits.Work_Model;
	//环路计数累加
	Control_Loop.Loop_Count ++;
	//环路计数到达限制清零
	if(Control_Loop.Loop_Count > 8)
	{
		Control_Loop.Loop_Count = 0;
	}
}

//PWM逻辑使能控制
void Enable_Logic_Control(void)
{
	//触发紧急停止不再考虑PWM使能状态是否改变，立即停止
	if(Control_Word.bits.Energency_Stop == 1)
	{
		HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);
		//关闭三相PWM输出
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
		//PWM使能状态置0
		Work_Status.bits.Enable_Status = 0;
	}
	//非紧急停止情况下，判断当前使能状态是否改变
	else if(Last_PWM_Enable != Control_Word.bits.PWM_Enable)
	{
		if(Control_Word.bits.PWM_Enable == 1)
		{
			//开启三相PWM输出
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
			//打开驱动模块PWM接受
			HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);
			//PWM使能状态置1
			Work_Status.bits.Enable_Status = 1;
		}
		else
		{
			//关闭驱动模块PWM接受
			HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);
			//关闭三相PWM输出
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			//PWM使能状态置0
			Work_Status.bits.Enable_Status = 0;
		}
		//更新当前PWM使能状态
		Last_PWM_Enable = Control_Word.bits.PWM_Enable;
	}
}


//应用算法
//最大转矩比控制MTPA
void MTPA_Control(FOC_Motor *motor)
{
	//根据输入合成电流Is,已知Ld、Lq和永磁体磁链计算最大力矩输出对应Id、Iq
	
}

//电流前馈解耦 根据上一次Iq、Id电流计算当前Uq、Ud前馈量,提升响应
void Current_Forward_Feedback(FOC_Motor *motor)
{
	//此处Id、Iq为上一次电流采样转换获得
	motor->Uq = motor->Uq + motor->Elecrical_Speed * (motor->Flux_Linkage + motor->Ld * motor->Id);
	motor->Ud = motor->Ud - motor->Elecrical_Speed * motor->Lq * motor->Id;
}

//电机控制死区补偿
void Dead_Time_Compensate(FOC_Motor *motor)
{
	//计算合成电流Is在静态坐标下离alpha轴角度，根据角度判断三相电流极性，再根据极性设置补偿时间增减
	//电流极性扇区判断
	uint8_t	dead_sector;

	if(motor->Ialpha > 0)
	{
		if((2 * motor->Ibeta - motor->Ialpha <= 0) && (2 * motor->Ibeta + motor->Ialpha > 0))
			dead_sector = 1;
		else if(2 * motor->Ibeta - motor->Ialpha > 0)
			dead_sector = 2;
		else
			dead_sector = 6;
	}
	else
	{
		if((2 * motor->Ibeta - motor->Ialpha >= 0) && (2 * motor->Ibeta + motor->Ialpha < 0))
			dead_sector = 4;
		else if(2 * motor->Ibeta - motor->Ialpha < 0)
			dead_sector = 5;
		else
			dead_sector = 3;
	}
	switch(dead_sector)
	{
		case 1:
			motor->Ta = motor->Ta + motor->Td;
		
			
		
	}
	
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


