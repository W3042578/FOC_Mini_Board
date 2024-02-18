
#include "foc.h"
#include "basic_function.h"
#include "parameter.h"



//定义电机结构体Motor1
FOC_Motor Motor1;		

//编码器输入数据统一为16位，进行查表
//电流12位采样，1A输入对应2048,1V电压控制为2048,保持单位统一
//电流Clark变换 等幅值变换
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

//SVPWM计算三相PWM时间输出比例
//4096为100%占空比输出标幺
//载波频率 = 开关管打开关闭频率（SVPWM 7段式）  基波频率 = 电机转速/（60/极对数）  一般载波频率：基波频率 = 10 ：1
void SVPWM(FOC_Motor *motor)
{
	uint8_t N_Sector;//扇区判断转换计数值
	//扇区判断
	motor->U1 = motor->Ubeta;
	motor->U2 = (3547 * motor->Ualph - 2048 * motor->Ubeta) >>12;  // 3547/4096=sqrt(3)/2
	motor->U3 = (-3547 * motor->Ualph - 2048 * motor->Ubeta) >>12;
	
	if(motor->U1 > 0)
		motor->Sa=1;
	else
		motor->Sa=0;
	if(motor->U2 > 0)
		motor->Sb=1;
	else
		motor->Sb=0;
	if(motor->U3 > 0)
		motor->Sc=1;
	else
		motor->Sc=0;
	N_Sector = motor->Sa + (motor->Sb<<1) + (motor->Sc<<2);
	//扇区电压作用量计算
	switch(N_Sector)
	{
		case 1: 
			motor->Sector = 2;
			motor->Tx = -motor->U2;									
			motor->Ty = -motor->U3;
		break;
		case 2:
			motor->Sector = 6;
			motor->Tx = -motor->U3;
			motor->Ty = -motor->U1;
		break;
		case 3:
			motor->Sector = 1;
			motor->Tx = motor->U2;
			motor->Ty = motor->U1;
		break;
		case 4:
			motor->Sector = 4;
			motor->Tx = -motor->U1;
			motor->Ty = -motor->U2;
		break;
		case 5:
			motor->Sector = 3;
			motor->Tx = motor->U1;
			motor->Ty = motor->U3;
		break;
		case 6:
			motor->Sector = 5;
			motor->Tx = motor->U3;
			motor->Ty = motor->U2;
		break;
		default:
			motor->Sector = 0;
			motor->Tx = 0;
			motor->Ty = 0;
		break;
	}
	//Tx,Ty转换为时间比例，使用Ts表示浮点1
	motor->Tx = (motor->Tx * 1774 / motor->Udc) >> 9; //1774>>10=1774/1024=sqrt(3) Tx按2048*实际电压值给入,Udc同比例放大2048,4096*sqrt(3)*Tx/Udc*2048
	motor->Ty = (motor->Ty * 1774 / motor->Udc) >> 9;
	//两相邻矢量作用时间限制，过调制限制或者弱磁MTPA
	if((motor->Tx + motor->Ty) > 3932)//调制限制 4096为1  0.96 * 4096 = 3932  限制满输出，留出采样时间
	{
		uint32_t data_32 = motor->Tx + motor->Ty;
		motor->Tx = motor->Tx * 3932 / data_32;
		motor->Ty = motor->Ty * 3932 / data_32;
	}
	//将比例转为由Ts计数值表示
	motor->Tx = (motor->Tx * motor->Ts) >> 12;
	motor->Ty = (motor->Ty * motor->Ts) >> 12;
}

//T0、Tx、Ty比例分配三相PWM计数值
void PWM_Time_Count(FOC_Motor *motor)
{
	//占空比模式三相PWM直接按照给入值计算输出比较值
	if(Control_Word.Work_Model == 2)
	{
		motor->Ta = 2.56 *(((100 - Control_Word.Duty_Model_A) * motor->Ts)>>8);
		motor->Tb = 2.56 *(((100 - Control_Word.Duty_Model_B) * motor->Ts)>>8);
		motor->Tc = 2.56 *(((100 - Control_Word.Duty_Model_C) * motor->Ts)>>8);
	}
	//非占空比模式三相根据Tx,Ty计算PWM输出比较值
	else
	{
		//T0不一定需要等于T7，发波方式和谐波成分与THD相关  https://blog.csdn.net/weixin_51553819/article/details/121856985
		//取T0 = T7 则 （Tx + Ty）/2 必在PWM 50% 输出点上 
		switch(motor->Sector)
		{
			case 1:
				motor->Ta = (motor->Ts - motor->Tx - motor->Ty)>>1;//长
				motor->Tb = (motor->Ts + motor->Tx - motor->Ty)>>1;//中
				motor->Tc = (motor->Ts + motor->Tx + motor->Ty)>>1;//短
			break;
			case 2:
				motor->Ta = (motor->Ts + motor->Tx - motor->Ty)>>1;//中
				motor->Tb = (motor->Ts - motor->Tx - motor->Ty)>>1;//长
				motor->Tc = (motor->Ts + motor->Tx + motor->Ty)>>1;//短
			break;
			case 3:
				motor->Ta = (motor->Ts + motor->Tx + motor->Ty)>>1;//短
				motor->Tb = (motor->Ts - motor->Tx - motor->Ty)>>1;//长
				motor->Tc = (motor->Ts + motor->Tx - motor->Ty)>>1;//中
			break;
			case 4:
				motor->Ta = (motor->Ts + motor->Tx + motor->Ty)>>1;//短
				motor->Tb = (motor->Ts + motor->Tx - motor->Ty)>>1;//中
				motor->Tc = (motor->Ts - motor->Tx - motor->Ty)>>1;//长
			break;
			case 5:
				motor->Ta = (motor->Ts + motor->Tx - motor->Ty)>>1;//中
				motor->Tb = (motor->Ts + motor->Tx + motor->Ty)>>1;//短
				motor->Tc = (motor->Ts - motor->Tx - motor->Ty)>>1;//长
			break;
			case 6:
				motor->Ta = (motor->Ts - motor->Tx - motor->Ty)>>1;//长
				motor->Tb = (motor->Ts + motor->Tx + motor->Ty)>>1;//短
				motor->Tc = (motor->Ts + motor->Tx - motor->Ty)>>1;//中
			break;
			default:
				motor->Ta = motor->Ts >>1;//默认输出50%占空比
				motor->Tb = motor->Ts >>1;
				motor->Tc = motor->Ts >>1;
			break;
		}
	}
	//避免0PWM输出时定时器模块工作不正常
	if(motor->Ta == 0)
		motor->Ta = 1;
	if(motor->Tb == 0)
		motor->Tb = 1;
	if(motor->Tc == 0)
		motor->Tc = 1;
}
//基本FOC控制
void FOC_Control(FOC_Motor *motor)
{ 
	//获取电角度
	Encoder_To_Electri_Angle(motor);
	//获取反馈电流Iq,Id
	Clark_Transform(motor);
	Park_Transform(motor);
	
	//模式处理
	Model_Control(motor);
	if(Control_Word.Work_Model != 2)//占空比模式不使用反park变换和SVPWM计算三相占空比
	{
		Inverse_Park_Transform(motor);
		SVPWM(motor);
	}
	PWM_Time_Count(motor);
}	
