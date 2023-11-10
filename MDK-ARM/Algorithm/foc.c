#include "foc.h"
#include "math.h"
#include "tim.h"
#include "spi.h"
#include "usart.h"
#include "stdio.h"
#include "sin_cos.h"
#include "main.h"
#include "usart_control.h"
#include "control_loop.h"
#include "object_commicate.h"
#include "hal_my.h"

//FOC_Motor 结构体变量初始化
//7极对数
//设置工作时间Ts = 2*(2+1)*(1499+1)/72M = 125us  中心对齐模式x2
//母线电压为 12V
//提升计算效率，浮点数全部转化为整数运算 除法移位运算
//放大倍率为 0XFFFF = 65535+1 16位
//将采集的12数据*4作为运算的16位数据，注意各个数据范围不要溢出
//取ia，ib为14位数据，则id，iq，Ud，Uq,Umax,Uref均为14位数据输出
void FOC_Motor_Init(FOC_Motor *motor)
{
	motor->Polar = 11;
	motor->Ts = 2130;//16k频率  满额2249  119*2的采样时间
	motor->Udc = 12;
	motor->Last_Encoder = 0;
	motor->Ts_Limit = motor->Ts ;
}
//电流Clark变换
void Clark_Transform(FOC_Motor *motor)
{
	motor->Ialph = motor->Ia;
	motor->Ibeta = (591 * motor->Ia + 1182 * motor->Ib)>>10;//591/1024=sqrt(3)/3
}

//电流Park变换
void Park_Transform(FOC_Motor *motor)
{
	//除以4096因为角度值是乘以4096后的整数值
	motor->Iq = (motor->Ibeta * motor->Cos_Angle - motor->Ialph * motor->Sin_Angle)>>12;
	motor->Id = (motor->Ibeta * motor->Sin_Angle + motor->Ialph * motor->Cos_Angle)>>12;
} 	
//电压Inverse Park变换
void Inverse_Park_Transform(FOC_Motor *motor)
{
	//除以4096因为角度值是乘以4096后的整数值
	motor->Ualph = (motor->Ud * motor->Cos_Angle - motor->Uq * motor->Sin_Angle)>>12; 
	motor->Ubeta = (motor->Ud * motor->Sin_Angle + motor->Uq * motor->Cos_Angle)>>12;
}
//SVPWM计算时间
//参考自 https://blog.csdn.net/hducollins/article/details/79260176
//参考自 https://blog.csdn.net/qq_41610710/article/details/120512746
void SVPWM(FOC_Motor *motor)
{
	//Tx与Ty已有过调制限制，此处弃用
	//幅值限制，避免过调制不成圆形,参考电压为 sqrt(3)/2 * 2/3 *Udc	= sqrt(3)*Udc/3
	//由三角形余弦定理  给出调制限制 六边形内切圆  
	//统一以 1024Dec:1V 作为标幺计算   
	// motor->Uref = (motor->Ualph * motor->Ualph + motor->Ubeta * motor->Ubeta)>>14;
	// if(motor->Uref > ((motor->Udc * motor->Udc)<<14))
	// {
	// 	motor->Ualph = (motor->Ualph * motor->Udc *motor->Udc)/motor->Uref;//正常限制调制圆应该是开根号取比例，此处节省计算使用平方取完比例会略小
	// 	motor->Ubeta = (motor->Ubeta * motor->Udc *motor->Udc)/motor->Uref;
	// }
	
	//扇区判断
	motor->U1 = motor->Ubeta;
	motor->U2 = (3547 * motor->Ualph - 2048 * motor->Ubeta) >>12;  // 3547/4096=sqrt(3)/2
	motor->U3 = (-3547 * motor->Ualph - 2048 * motor->Ubeta) >>12;
	
	if(motor->U1>0)
		motor->Sa=1;
	else
		motor->Sa=0;
	if(motor->U2>0)
		motor->Sb=1;
	else
		motor->Sb=0;
	if(motor->U3>0)
		motor->Sc=1;
	else
		motor->Sc=0;
	motor->Sector_Add = motor->Sa + (motor->Sb<<1) + (motor->Sc<<2);
	
	//计算矢量作用时间,注意motor->Tx，motor->Ty先后发生对应的电压矢量
	//！！改用定时器最大计数值进行计算，标幺化
	motor->m32 =(1774 * motor->Ts/motor->Udc) >>10;// motor->Udc 没有放大，1774/1024=sqrt(3) 将m放大14位使m与U1,U2,U3同放大单位
	switch(motor->Sector_Add)
	{
		case 1: 
			motor->Sector_Actual = 2;
			motor->Tx = (-motor->U2 * motor->m32) >>14;									//右移14位等价于除以16384
			motor->Ty = (-motor->U3 * motor->m32) >>14;
		break;
		case 2:
			motor->Sector_Actual = 6;
			motor->Tx = (-motor->U3 * motor->m32) >>14;
			motor->Ty = (-motor->U1 * motor->m32) >>14;
		break;
		case 3:
			motor->Sector_Actual = 1;
			motor->Tx = (motor->U2 * motor->m32) >>14;
			motor->Ty = (motor->U1 * motor->m32) >>14;
		break;
		case 4:
			motor->Sector_Actual = 4;
			motor->Tx = (-motor->U1 * motor->m32) >>14;
			motor->Ty = (-motor->U2 * motor->m32) >>14;
		break;
		case 5:
			motor->Sector_Actual = 3;
			motor->Tx = (motor->U1 * motor->m32) >>14;
			motor->Ty = (motor->U3 * motor->m32) >>14;
		break;
		case 6:
			motor->Sector_Actual = 5;
			motor->Tx = (motor->U3 * motor->m32) >>14;
			motor->Ty = (motor->U2 * motor->m32) >>14;
		break;
		default:
			motor->Sector_Actual = 0;
			motor->Tx = 0;
			motor->Ty = 0;
			break;
	}
	if((motor->Tx + motor->Ty)>motor->Ts_Limit)//调制限制 相似三角
	{
		motor->temp16 =motor->Tx + motor->Ty;
		motor->Tx = motor->Tx * motor->Ts_Limit / motor->temp16;
		motor->Ty = motor->Ty * motor->Ts_Limit / motor->temp16;
	}
		switch(motor->Sector_Actual)
	{
		case 1:
			motor->Ta =(motor->Ts + motor->Tx + motor->Ty)>>1;	//右移1位等价于除以2
			motor->Tb =(motor->Ts - motor->Tx + motor->Ty)>>1;
			motor->Tc =(motor->Ts - motor->Tx - motor->Ty)>>1;
		break;
		case 2:
			motor->Ta =(motor->Ts - motor->Tx + motor->Ty)>>1;
			motor->Tb =(motor->Ts + motor->Tx + motor->Ty)>>1;
			motor->Tc =(motor->Ts - motor->Tx - motor->Ty)>>1;
		break;
		case 3:
			motor->Ta =(motor->Ts - motor->Tx - motor->Ty)>>1;
			motor->Tb =(motor->Ts + motor->Tx + motor->Ty)>>1;
			motor->Tc =(motor->Ts - motor->Tx + motor->Ty)>>1;
		break;
		case 4:
			motor->Ta =(motor->Ts - motor->Tx - motor->Ty)>>1;
			motor->Tb =(motor->Ts - motor->Tx + motor->Ty)>>1;
			motor->Tc =(motor->Ts + motor->Tx + motor->Ty)>>1;
		break;
		case 5:
			motor->Ta =(motor->Ts - motor->Tx + motor->Ty)>>1;
			motor->Tb =(motor->Ts - motor->Tx - motor->Ty)>>1;
			motor->Tc =(motor->Ts + motor->Tx + motor->Ty)>>1;
		break;
		case 6:
			motor->Ta =(motor->Ts + motor->Tx + motor->Ty)>>1;
			motor->Tb =(motor->Ts - motor->Tx - motor->Ty)>>1;
			motor->Tc =(motor->Ts - motor->Tx + motor->Ty)>>1;
		break;
		default:
			motor->Ta =0;
			motor->Tb =0;
			motor->Tc =0;
			break;
	}
}

//更新三路定时器ccr触发数值
//参考高级定时器生成六路互补带死区pwm ：https://blog.csdn.net/wsq_666/article/details/126555569
void Set_Motor_Channel_Time(FOC_Motor *motor)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->Ta);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->Tb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motor->Tc);	
}

//获取编码器角度值
//单位：编码值 2~14   14位编码器
//！！改进要求DMA传输  尽可能缩短通讯时间  作编码器角度迟滞补偿
uint16_t Get_Angle_MT6813(FOC_Motor *motor)
{
	uint16_t Angle;
	uint16_t Angle_Transfer[2];
	HAL_SPI_TransmitReceive_DMA(&hspi1,(uint8_t*)Tx_Encoder,(uint8_t*)Rx_Encoder,2);   //数据发送接占用较多时间，实际时间需要控制在 62.5/2 - 6usADC  = 25us内  
//	HAL_SPI_TransmitReceive_DMA(&hspi1,&Tx_Encoder[2],&Rx_Encoder[2],2);
	Rx_Encoder[0] = Rx_Encoder[0] << 8;
	Rx_Encoder[1] = Rx_Encoder[1] >> 8;
	Transfer1[0] = Rx_Encoder[0];
	Transfer1[1] = Rx_Encoder[1];

	//两组数据合并得出编码值,并给出编码器磁强度状态信号：1为磁场强度不足，0为正常工作
	Angle_Transfer[0] = ((uint16_t)Rx_Encoder[0]) >> 2;   //数据左移6位
	(*motor).Encoder_Tag =(uint8_t)((0x0002 & Rx_Data[1]) >>1);  		//获取第二位磁状态
	Angle_Transfer[1] = ((uint16_t)Rx_Encoder[1]) >> 2;			//数据左移2位
	Angle = Angle_Transfer[0] + Angle_Transfer[1];
	
	Transfer1[2] = Angle & 0x3FFF;//14位编码器输出范围限制
	return Angle;
}
//获取编码器修正值
void Encoder_Offest(FOC_Motor *motor)
{
	uint32_t Add_Encoder_Offest;
	uint16_t Number_Encoder_Offest;
	Number_Encoder_Offest = 32;
	Add_Encoder_Offest	= 0;
	
 	motor->Ualph = 49152;//Ualph = 3V
	motor->Ubeta = 0;
	SVPWM(motor);
	Set_Motor_Channel_Time(motor);
	
	//使能drv8313
	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);	
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	HAL_Delay(1000);
	//平均减小误差
	for(uint8_t i = 1;i<=Number_Encoder_Offest;i++)
		{
			Add_Encoder_Offest = Add_Encoder_Offest + 16384 - Get_Angle_MT6813(motor);
		}
		
	motor->Encoder_Offest = Add_Encoder_Offest>>5;
		
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	
	//关闭使能drv8313
	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);	
}
//获取两相电流采样修正值  包含偏置电压
void ADC_Current_Offest(FOC_Motor *motor)
{
	uint32_t Add_ADC_Offect_U,Add_ADC_Offect_V;
	uint16_t	Number_ADC_Offect;
	
	Add_ADC_Offect_U = Add_ADC_Offect_V = 0;
	Number_ADC_Offect = 32;
	motor->Ia_Offect = 0;				//校准值置零，避免采样回调函数中修正值影响直接采得的数据
	motor->Ib_Offect = 0;
	
	motor->Ualph = 0;//Ualph = 0V
	motor->Ubeta = 0;
	SVPWM(motor);
	Set_Motor_Channel_Time(motor);
	
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
		
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
		
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	//关闭使能drv8313
	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);	
}
//获取修正电气角度并转换为sin，cos值，根据编码器修正角度计算速度
void Get_Electrical_Angle(FOC_Motor *motor)
{
	motor->Encoder = 16384 - Get_Angle_MT6813(motor);
	if(motor->Encoder - motor->Encoder_Offest < 0)
		motor->Encoder = motor->Encoder - motor->Encoder_Offest + 16384;
	else
		motor->Encoder = motor->Encoder - motor->Encoder_Offest;
	//获取修正机械角度对应的电气角度（单位：度数）
	motor->Mechanical_Angle = (360*motor->Encoder)>>14;
	motor->Elecrical_Angle = (motor->Polar * motor->Mechanical_Angle)%360;//丢失编码器精度，角度应该到分
	//由角度值获取对应sin、cos乘以4096值
	motor->Sin_Angle = Get_Sin(motor->Elecrical_Angle);
	motor->Cos_Angle = Get_Cos(motor->Elecrical_Angle);

	//计算速度  现在角度减去过去角度
	motor->Encoder_Speed_Angle_Loop = motor->Encoder - motor->Last_Encoder_Loop;
	if(Control_Data.Work_Direction == 0)//正转速度必为正
    {
      if(motor->Encoder_Speed_Angle_Loop < 0)
        motor->Encoder_Speed_Angle_Loop = 16384 + motor->Encoder_Speed_Angle_Loop;
    }
	else                 	            //反转速度必为负
    {
      if(Motor1.Encoder_Speed_Angle > 0)
		  	motor->Encoder_Speed_Angle_Loop = motor->Encoder_Speed_Angle_Loop - 16384;
    }
	motor->Speed_Filter_Loop = (motor->Speed_Filter_Loop + 3 * motor->Encoder_Speed_Angle_Loop)>>2;
    //此次速度作为下一次的过往速度
	motor->Last_Encoder_Loop = motor->Encoder_Speed_Angle_Loop;
}
//FOC控制
//根据控制字模式选择工作模式和工作电流、电压、速度、位置要求，并判断是否使能
void FOC_Control(FOC_Motor *motor)
{
	
	switch(Control_Data.Work_Model)
	{
		case 0x00:																			//开环控制
			if(Control_Data.Work_Direction)
			motor->Uq = -16384 * Control_Data.Open_Loop_Voltage;
			else
			motor->Uq = 16384 * Control_Data.Open_Loop_Voltage;
			motor->Ud = 0;
			break;
			
		case 0x01:																			//电流环
			Clark_Transform(motor);
			Park_Transform(motor);
		
			//Iq PID
			FOC_PID_Current_Iq.Expect = Control_Loop.Target_Current;
			FOC_PID_Current_Iq.Feed_Back = (*motor).Iq;
			PID_Control(&FOC_PID_Current_Iq);
			(*motor).Uq = FOC_PID_Current_Iq.Output;  
		
			//Id PID
			FOC_PID_Current_Id.Expect = 0;
			FOC_PID_Current_Id.Feed_Back = (*motor).Id;
			PID_Control(&FOC_PID_Current_Id);
			(*motor).Ud = FOC_PID_Current_Id.Output<<3;
			break;
		
		case 0x10:																			//速度环
			if(Loop_Time_Count == 0)
			{
				Loop_Time_Count = 2;
				FOC_PID_Speed.Expect = Control_Loop.Target_Speed;
				//速度采样判断逻辑:在8k频率中能够分辨出速度变化则使用8k速度环获取数据计算速度值，否则低速下使用1k定时器获取数据计算速度值
				//乘数过大带来速度计算误差放大
				if(Motor1.Encoder_Speed_Angle_Loop < 20)
					FOC_PID_Speed.Feed_Back = (1875 * (*motor).Encoder_Speed_Angle)>>9; //1875/512 = 60*1000/16384  1ms转化为rpm单位
				else
					FOC_PID_Speed.Feed_Back = (1875 * (*motor).Encoder_Speed_Angle_Loop)>>6; //8*1875/512 = 8*60*1000/16384  0.125ms转化为rpm单位
				PID_Control(&FOC_PID_Speed);
			}
			Clark_Transform(motor);
			Park_Transform(motor);
		
			//Iq PID
			FOC_PID_Current_Iq.Expect = FOC_PID_Speed.Output;
			FOC_PID_Current_Iq.Feed_Back = (*motor).Iq;
			PID_Control(&FOC_PID_Current_Iq);
			(*motor).Uq = FOC_PID_Current_Iq.Output;
		
			//Id PID
			FOC_PID_Current_Id.Expect = 0;
			FOC_PID_Current_Id.Feed_Back = (*motor).Id;
			PID_Control(&FOC_PID_Current_Id);
			(*motor).Ud = FOC_PID_Current_Id.Output;
			
			Loop_Time_Count--;
			break;
		case 0x11:																		//位置环
			if(Loop_Time_Count == 0)
			{
				Loop_Time_Count = 4;
				FOC_PID_Position.Expect = Control_Loop.Target_Position;
				FOC_PID_Position.Feed_Back = (*motor).Encoder;
				PID_Control(&FOC_PID_Position);
			}
			FOC_PID_Speed.Expect = FOC_PID_Position.Output;
			FOC_PID_Speed.Feed_Back = (1875 * (*motor).Encoder_Speed_Angle)>>9; //1875/512 = 60*1000/16384  转化为rpm单位
			PID_Control(&FOC_PID_Speed);
		
			Clark_Transform(motor);
			Park_Transform(motor);
			
			//Iq PID
			FOC_PID_Current_Iq.Expect = FOC_PID_Speed.Output;
			FOC_PID_Current_Iq.Feed_Back = (*motor).Iq;
			PID_Control(&FOC_PID_Current_Iq);
			(*motor).Uq = FOC_PID_Current_Iq.Output;
		
			//Id PID
			FOC_PID_Current_Id.Expect = 0;
			FOC_PID_Current_Id.Feed_Back = (*motor).Id;
			PID_Control(&FOC_PID_Current_Id);
			(*motor).Ud = FOC_PID_Current_Id.Output;
			Loop_Time_Count--;
			
			break;
		default:
			Error_Message.bits.Control_Loop_Error = 1;
			break;
	}
	Inverse_Park_Transform(motor);
	SVPWM(motor);
	Set_Motor_Channel_Time(motor);
}
