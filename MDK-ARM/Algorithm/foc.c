
#include "foc.h"
#include "basic_function.h"
#include "parameter.h"



//定义电机结构体Motor1
FOC_Motor Motor1;		

//编码器输入数据统一为16位，进行查表
//电流12位采样
//电流Clark变换 等幅值变换
//Q7数据格式：int16位数据、1符号位、8整数位、7小数位
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

//SVPWM计算三相PWM时间计数
//载波频率 = 开关管打开关闭频率（SVPWM 7段式）  基波频率 = 电机转速/（60/极对数）  一般载波频率：基波频率 = 10 ：1
void SVPWM(FOC_Motor *motor)
{
	
	int16_t u1,u2,u3;			//SVPWM扇区判断电压
	int16_t u_lead,u_backward;	//矢量圆分解先后电压
	uint16_t time_lead,time_backward;	//矢量圆分解电压作用时间
	uint8_t n_sector,sector;	//扇区判断数
	uint8_t sector_single = 0;	//扇区判断标志位
	
	//扇区判断
	u1 = motor->Ubeta;
	u2 = (3547 * motor->Ualph - 2048 * motor->Ubeta) >>12;  // 3547/4096=sqrt(3)/2
	u3 = (-3547 * motor->Ualph - 2048 * motor->Ubeta) >>12;
	
	if(u1 > 0)
		sector_single |= 0x01;
	else
		sector_single &= 0xFE;
	if(u2 > 0)
		sector_single |= 0x02;
	else
		sector_single &= 0xFD;
	if(u3 > 0)
		sector_single &= 0x04;
	else
		sector_single &= 0xFB;
	//扇区电压作用量计算
	switch(n_sector)
	{
		case 1: 
			sector = 2;
			u_lead = -u2;									
			u_backward = -u3;
		break;
		case 2:
			sector = 6;
			u_lead = -u3;
			u_backward = -u1;
		break;
		case 3:
			sector = 1;
			u_lead = u2;
			u_backward = u1;
		break;
		case 4:
			sector = 4;
			u_lead = -u1;
			u_backward = -u2;
		break;
		case 5:
			sector = 3;
			u_lead = u1;
			u_backward = u3;
		break;
		case 6:
			sector = 5;
			u_lead = u3;
			u_backward = u2;
		break;
		default:
			sector = 0;
			u_lead = 0;
			u_backward = 0;
		break;
	}
	//使用Ts计数值表示满占空比
	time_lead = ((u_lead * motor->Ts * 1774 / motor->Udc) >> 10) >> (_INIT_SCALE); //1774>>10=1774/1024=sqrt(3)
	time_backward = ((u_backward * motor->Ts * 1774 / motor->Udc) >> 10) >> (_INIT_SCALE);
	//两相邻矢量作用时间限制，过调制限制或者弱磁MTPA
	if((time_lead + time_backward) > (0.96 * motor->Ts))//调制限制 4096为1  0.96 * 4096 = 3932  限制满输出，留出采样时间
	{
		uint16_t data_16 = time_lead + time_backward;
		time_lead = time_lead  / data_16;
		time_backward = time_backward  / data_16;
	}

	//分配三相PWM计数值
	//T0不一定需要等于T7，发波方式和谐波成分与THD相关  https://blog.csdn.net/weixin_51553819/article/details/121856985
	//取T0 = T7 则 （Tx + Ty）/2 必在PWM 50% 输出点上 
	switch(sector)
	{
		case 1:
			motor->Ta = (motor->Ts - time_lead - time_backward)>>1;//长
			motor->Tb = (motor->Ts + time_lead - time_backward)>>1;//中
			motor->Tc = (motor->Ts + time_lead + time_backward)>>1;//短
		break;
		case 2:
			motor->Ta = (motor->Ts + time_lead - time_backward)>>1;//中
			motor->Tb = (motor->Ts - time_lead - time_backward)>>1;//长
			motor->Tc = (motor->Ts + time_lead + time_backward)>>1;//短
		break;
		case 3:
			motor->Ta = (motor->Ts + time_lead + time_backward)>>1;//短
			motor->Tb = (motor->Ts - time_lead - time_backward)>>1;//长
			motor->Tc = (motor->Ts + time_lead - time_backward)>>1;//中
		break;
		case 4:
			motor->Ta = (motor->Ts + time_lead + time_backward)>>1;//短
			motor->Tb = (motor->Ts + time_lead - time_backward)>>1;//中
			motor->Tc = (motor->Ts - time_lead - time_backward)>>1;//长
		break;
		case 5:
			motor->Ta = (motor->Ts + time_lead - time_backward)>>1;//中
			motor->Tb = (motor->Ts + time_lead + time_backward)>>1;//短
			motor->Tc = (motor->Ts - time_lead - time_backward)>>1;//长
		break;
		case 6:
			motor->Ta = (motor->Ts - time_lead - time_backward)>>1;//长
			motor->Tb = (motor->Ts + time_lead + time_backward)>>1;//短
			motor->Tc = (motor->Ts + time_lead - time_backward)>>1;//中
		break;
		default:
			motor->Ta = motor->Ts >>1;//默认输出50%占空比
			motor->Tb = motor->Ts >>1;
			motor->Tc = motor->Ts >>1;
		break;
	}
	
	//电机控制死区补偿
	// Dead_Time_Compensate(motor);

	//未输出状态避免死区补偿影响
	// if((sector == 0) || (sector == 7))
	// {
	// 	motor->Ta = motor->Ts >>1;//默认输出50%占空比
	// 	motor->Tb = motor->Ts >>1;
	// 	motor->Tc = motor->Ts >>1;	
	// }
	

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
	uint8_t model;
	//获取电角度
	Encoder_To_Electri_Angle(motor);
	//获取反馈电流Iq,Id
	Clark_Transform(motor);
	//换算获取Iq,Id
	Park_Transform(motor);
	
	//模式处理
	model = Model_Control(motor);
	if(model != 0x02)//占空比模式不使用反park变换和SVPWM计算三相占空比
	{
		Inverse_Park_Transform(motor);
		SVPWM(motor);
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

	if(motor->Ialph > 0)
	{
		if((2 * motor->Ibeta - motor->Ialph <= 0) && (2 * motor->Ibeta + motor->Ialph > 0))
			dead_sector = 1;
		else if(2 * motor->Ibeta - motor->Ialph > 0)
			dead_sector = 2;
		else
			dead_sector = 6;
	}
	else
	{
		if((2 * motor->Ibeta - motor->Ialph >= 0) && (2 * motor->Ibeta + motor->Ialph < 0))
			dead_sector = 4;
		else if(2 * motor->Ibeta - motor->Ialph < 0)
			dead_sector = 5;
		else
			dead_sector = 3;
	}
	//死区时间小于三相占空比才作用
	if((motor->Ta > motor->Td) && (motor->Tb > motor->Td) && (motor->Tc > motor->Td))
	{
		switch(dead_sector)
		{
		case 1:
			motor->Ta = motor->Ta + (motor->Td >> 1);
			motor->Tb = motor->Tb - (motor->Td >> 1);
			motor->Tc = motor->Tc - (motor->Td >> 1);
		break;
		case 2:
			motor->Ta = motor->Ta + (motor->Td >> 1);
			motor->Tb = motor->Tb + (motor->Td >> 1);
			motor->Tc = motor->Tc - (motor->Td >> 1);
		break;
		case 3:
			motor->Ta = motor->Ta - (motor->Td >> 1);
			motor->Tb = motor->Tb + (motor->Td >> 1);
			motor->Tc = motor->Tc - (motor->Td >> 1);
		break;
		case 4:
			motor->Ta = motor->Ta - (motor->Td >> 1);
			motor->Tb = motor->Tb + (motor->Td >> 1);
			motor->Tc = motor->Tc + (motor->Td >> 1);
		break;
		case 5:
			motor->Ta = motor->Ta - (motor->Td >> 1);
			motor->Tb = motor->Tb - (motor->Td >> 1);
			motor->Tc = motor->Tc + (motor->Td >> 1);
		break;
		case 6:
			motor->Ta = motor->Ta + (motor->Td >> 1);
			motor->Tb = motor->Tb - (motor->Td >> 1);
			motor->Tc = motor->Tc + (motor->Td >> 1);
		break;
		default:
			
		break;
		}
	}
}
