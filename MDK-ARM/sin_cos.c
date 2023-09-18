
#include "main.h"
#include "sin_cos.h"

//查表获取sin、cos值
int SIN_COS_TABLE[91] = {0,71,142,214,285,356,428,499,570,640,711,781,851,921,990,1060,1128,1197,1265,
	1333,1400,1467,1534,1600,1665,1731,1795,1859,1922,1985,2047,2109,2170,2230,2290,2349,2407,2465,
	2521,2577,2632,2687,2740,2793,2845,2896,2946,2995,3043,3091,3137,3183,3227,3271,3313,3355,3395,
	3435,3473,3510,3547,3582,3616,3649,3681,3712,3741,3770,3797,3823,3848,3872,3895,3917,3937,3956,
	3974,3991,4006,4020,4033,4045,4056,4065,4073,4080,4086,4090,4093,4095,4096};	

//获取输入角度值对应sin值，输出sin值与4096乘积整数
//输入整形数据uint16_t 范围为 0-360 角度值DEG不是RAD
int16_t Get_Sin(uint16_t x)
{
	if(x <= 90)
		return SIN_COS_TABLE[x];
	else if(x > 90 && x <= 180)
		return SIN_COS_TABLE[90 - (x - 90)];
	else if(x > 180 && x <= 270)
		return -SIN_COS_TABLE[x - 180];
	else if (x >270 && x <= 360)
		return -SIN_COS_TABLE[90 - (x - 270)];
	else 
		return 0;
	
}

//获取输入角度值对应cos值，输出cos值与4096乘积整数
int16_t Get_Cos(uint16_t x)
{
	if(x <= 90)
		return SIN_COS_TABLE[90 - x];
	else if(x > 90 && x <= 180)
		return -SIN_COS_TABLE[x - 90];
	else if(x > 180 && x <= 270)
		return -SIN_COS_TABLE[90 - (x - 180)];
	else if (x >270 && x <= 360)
		return SIN_COS_TABLE[x - 270];
	else 
		return 0;
	
}
