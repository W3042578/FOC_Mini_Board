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

//底层配置
void STM32_Infrastructure_Init(void)	//底层初始化配置
{
	HAL_ADCEx_Calibration_Start(&hadc1);	//校准ADC采样,校准完成后仍保持adc使能
	HAL_ADCEx_Calibration_Start(&hadc2);	//同步注入采样中adc1为主采样器，adc2为从配置器，因此触发adc1即可触发adc2

	HAL_ADCEx_InjectedStart(&hadc2);	//开启注入组采样，注意注入采样控制中断的处理函数中会关闭注入中断使能位
	HAL_ADCEx_InjectedStart_IT(&hadc1);	//adc使能打开仍需要HAL_ADCEx_InjectedStart函数开始注入采样

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);	//开启cc4比较通道触发adc注入采样

	HAL_TIM_Base_Start_IT(&htim2);	//定时器2开启1ms中断

	HAL_UART_Receive_DMA(&huart1,(uint8_t *)&Rx_Data,RX_BUFF_LONG);	//开启串口DMA发送和接受

	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);	//使能串口空闲中断
	
	HAL_GPIO_WritePin(Power_Reset_GPIO_Port,Power_Reset_Pin,GPIO_PIN_SET);	//使能驱动模块
	
}

//STM32 HAL 三相PWM比较值设置输出
void STM32_HAL_PWM_SET_Compare(_FOC_Motor *motor)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->Ta);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->Tb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motor->Tc);
}

//电流采样
void ADC_Current_Offest(_FOC_Motor *motor)	//获取两相电流采样修正值  包含偏置电压
{
	int32_t	Add_ADC_Offect_U,Add_ADC_Offect_V;
	int16_t	Number_ADC_Offect;
	Control_Status.Work_Status.bits.Offest_Current = 1;
	Add_ADC_Offect_U = Add_ADC_Offect_V = 0;
	Number_ADC_Offect = 32;
	motor->Ia_Offect = 0;				//校准值置零，避免采样回调函数中修正值影响直接采得的数据
	motor->Ib_Offect = 0;

	HAL_Delay(1000);
	//平均减小误差
	for(uint8_t i = 0;i<Number_ADC_Offect;i++)
	{
		//累加考虑到电流获取负号，负负得正
		Add_ADC_Offect_U = Add_ADC_Offect_U + (Motor1.Uadc - 2048);
		Add_ADC_Offect_V = Add_ADC_Offect_V + (Motor1.Vadc - 2048);
		HAL_Delay(2);//延迟2ms 62.5us获取一次电流采样值，确保2ms内采样值至少更新一次
	}
	motor->Ia_Offect = 2048 + (Add_ADC_Offect_U >> 5);
	motor->Ib_Offect = 2048 + (Add_ADC_Offect_V >> 5);
	motor->Ia = motor->Ib = 0;	
}

//编码器&角度
void Encoder_To_Electri_Angle(_FOC_Motor *motor)	//获取编码器角度并转换为电角度
{
	int32_t offest_angle;
	_Control_Status *status = &Control_Status;

	//角度获取
	Encoder_Get_Angle(&Encoder1); //获取编码器角度，速度，位置	
	
	//电机原点位置对齐alpha轴
	if(Encoder1.Encoder_Pulse < motor->Initial_Angle_Offset)
		motor->Mechanical_Angle = Encoder1.Encoder_Pulse - motor->Initial_Angle_Offset + 65535;
	else
		motor->Mechanical_Angle = Encoder1.Encoder_Pulse - motor->Initial_Angle_Offset;
	
	//电机速度补偿计算用角度
	// offest_angle = motor->Mechanical_Angle + motor->Speed_Angle;
	offest_angle = motor->Mechanical_Angle;
	if(offest_angle > 65535)
	{
		offest_angle = offest_angle - 65535;
	}
	else if(offest_angle < 0)
	{
		offest_angle = offest_angle + 65535;
	}
	else if(offest_angle < - 65535)
	{
		//速度错误
	}
	//使用与运算快速取余 t % 2`(n) 等价于 t & (2`(n) - 1)
	if(status->Work_Status.bits.Offest_Encoder != 1)	//编码器校准对齐模式下使用虚拟电角度赋值
		motor->Elecrical_Angle = (motor->Polar * offest_angle) & 0xFFFF;
	//查表获取电角度对应三角函数值
	motor->Sin_Angle = SIN_COS_TABLE[(motor->Elecrical_Angle >> 7)];
	motor->Cos_Angle = SIN_COS_TABLE[((motor->Elecrical_Angle >> 7)+128) & 0x1ff];
}

//应用层功能

uint8_t Model_Control(_Control_Data *Data,_Control_Status *Status)
{
	uint8_t		work_model,sub_work_model;	//工作模式判断

	work_model = Data->Control_Word.bits.Work_Model_Buffer;
	sub_work_model = Data->Control_Word.bits.Sub_Work_Model_Buffer;
	
	//根据控制字判断工作环
	switch(work_model)
	{	
		case OPEN_LOOP:		//开环控制
			Data->Control_Source.Position_Source = NULL_MODEL;
			Data->Control_Source.Speed_Source = NULL_MODEL;
			Data->Control_Source.Current_Source = NULL_MODEL;
			switch (sub_work_model)
			{
				case OPEN_VOLTAGE:		//开环电压
					Data->Control_Source.Voltage_Source = OPEN_VOLTAGE;
				break;
				case DUTY_CONTROL:		//占空比控制
					Data->Control_Source.Voltage_Source = DUTY_CONTROL;
				break;
				case EOCODER_OFFEST:	//编码器修正
					Data->Control_Source.Voltage_Source = EOCODER_OFFEST;
				break;
				default:				//默认置空
					Data->Control_Source.Voltage_Source = NULL_MODEL;
				break;
			}
		break;
		case SENSELESS:		//无感控制
		
		break;
		case POSITION_LOOP:	//位置环控制
			Data->Control_Source.Speed_Source = POSITION_LOOP;
			Data->Control_Source.Current_Source = POSITION_LOOP;
			Data->Control_Source.Voltage_Source = POSITION_LOOP;
			switch (sub_work_model)
			{
				case NORMAL_CONTROL:		//常规控制
					Data->Control_Source.Position_Source = NORMAL_CONTROL;
				break;
				case DEBUG_ENGINE:			//调试控制
					Data->Control_Source.Position_Source = DEBUG_ENGINE;
				break;
				default:
					Data->Control_Source.Position_Source = NULL_MODEL;
				break;
			}
		break;
		case SPEED_LOOP:	//速度环控制
			Data->Control_Source.Position_Source = NULL_MODEL;
			Data->Control_Source.Current_Source = SPEED_LOOP;
			Data->Control_Source.Voltage_Source = SPEED_LOOP;
			switch (sub_work_model)
			{
				case NORMAL_CONTROL:		//常规控制
					Data->Control_Source.Speed_Source = NORMAL_CONTROL;
				break;
				case DEBUG_ENGINE:			//调试控制
					Data->Control_Source.Speed_Source = DEBUG_ENGINE;
				break;
				default:
					Data->Control_Source.Speed_Source = NULL_MODEL;
				break;
			}
		break;
		case CURRENT_LOOP: //电流环控制
			Data->Control_Source.Position_Source = NULL_MODEL;
			Data->Control_Source.Speed_Source = NULL_MODEL;
			Data->Control_Source.Voltage_Source = CURRENT_LOOP;
			switch (sub_work_model)
			{
				case NORMAL_CONTROL:		//常规控制
					Data->Control_Source.Current_Source = NORMAL_CONTROL;
				break;
				case MTPA_CONTROL:			//最大转矩比控制
					Data->Control_Source.Current_Source = MTPA_CONTROL;
				break;
				case DEBUG_ENGINE:			//调试控制
					Data->Control_Source.Current_Source = DEBUG_ENGINE;
				break;
				default:
					Data->Control_Source.Current_Source = NULL_MODEL;
				break;
			}
		break;
		default:
			Data->Control_Source.Position_Source = NULL_MODEL;
			Data->Control_Source.Speed_Source = NULL_MODEL;
			Data->Control_Source.Current_Source = NULL_MODEL;
			Data->Control_Source.Voltage_Source = NULL_MODEL;
		break;
	}
	if(Control_Status.Work_Status.bits.Enable_Status == 0)	//非使能条件下
	{
		if((Control_Data.Control_Word.bits.Work_Model_Buffer != Control_Data.Control_Word.bits.Work_Model) || \
		(Control_Data.Control_Word.bits.Sub_Work_Model_Buffer != Control_Data.Control_Word.bits.Sub_Work_Model))
		{	//控制模式变更
			Control_Data.Control_Word.bits.Work_Model_Buffer = Control_Data.Control_Word.bits.Work_Model;	//保存上一次控制模式
			Control_Data.Control_Word.bits.Sub_Work_Model_Buffer = Control_Data.Control_Word.bits.Sub_Work_Model;
		}
	}
	return work_model;	
}

//PWM逻辑使能控制
void Enable_Logic_Control(void)
{
	//触发紧急停止不再考虑PWM使能状态是否改变，立即停止
	if(Control_Data.Control_Word.bits.Energency_Stop == 1)
	{
		HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);
		//关闭三相PWM输出
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
		//PWM使能状态置0
		Control_Status.Work_Status.bits.Enable_Status = 0;
	}
	//非紧急停止情况下，判断当前使能状态是否改变
	else if(Control_Status.Work_Status.bits.Enable_Status != Control_Data.Control_Word.bits.PWM_Enable)
	{
		if(Control_Data.Control_Word.bits.PWM_Enable == 1)
		{
			//开启三相PWM输出
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
			//打开驱动模块PWM接受
			HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);
			//PWM使能状态置1
			Control_Status.Work_Status.bits.Enable_Status = 1;
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
			Control_Status.Work_Status.bits.Enable_Status = 0;
		}
	}
}


//1ms速度计算 M法测量
void Speed_1MS(void)
{
	int32_t Speed;
	// //判断是否首次进入1ms中断
	// if(Work_Status.bits.Interrupt_1MS_Init == 0)
	// {
	// 	Work_Status.bits.Interrupt_1MS_Init = 1;
	// 	//令过去位置对于当前位置，避免起步错误速度
	// 	Motor_Status1.Last_Encoder_Position = Encoder1.Encode_Position;
	// }
	// //1ms速度等于 当前位置 - 1ms前位置
	// Speed = Encoder1.Encode_Position - Motor_Status1.Last_Encoder_Position;
	// //对单次速度变化作限制
	// if(Motor_Status1.Last_1MS_Speed - Speed > 250)
	// 	Speed = Motor_Status1.Last_1MS_Speed;
	// if(Speed - Motor_Status1.Last_1MS_Speed > 250)
	// 	Speed = Motor_Status1.Last_1MS_Speed;

	// //编码器速度范围限制,超出限制以上一次速度代替
	// if((Speed > 32763) || (Speed < -32763))
	// 	Speed = Motor_Status1.Last_1MS_Speed;
	
	// //更新过去1ms时间编码器位置、1ms编码器速度
	// Motor_Status1.Last_Encoder_Position = Encoder1.Encode_Position;
	// Motor_Status1.Last_1MS_Speed = Speed;
	
	// //输出直接编码器速度
	// // Encoder1.Encoder_1MS_Speed = Speed;

	// //1ms速度计算1个电流环周期机械角速度并乘2
	// //乘3因为当前周期获取得的速度为上一个周期发送指令获取得，在下一周期生效
	// Motor1.Speed_Angle = (3 * Speed) >> 4;
}

//外部输入输出数据、温度保护、电机状态保护、速度计算处理
void Interrupt_1MS(void)
{
	

}


