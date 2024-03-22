
#include "filter.h"

#include "table.h"

//数字滤波器
//滑动滤波器	滑动窗口 = 2*滑动倍数 + 1	小于滑动窗口g个数的两端数据不作处理
//data_in：输入数据指针	data_out：输出数据指针	data_count：数据个数	sliding_rate：滑动倍数  
void Sliding_Filter(int16_t * data_in, int16_t * data_out, int16_t data_count, uint8_t sliding_rate)
{
	int32_t sum, average;
	for(uint16_t i = 0; i < (data_count - sliding_rate); i++)
	{
		if(i < sliding_rate)	//小于滑动窗口个数的两端数据不作处理
		{
			*(data_out + i) = *(data_in + i);
		}
		else	//开始滑动滤波
		{
			for(uint16_t t = i - sliding_rate; t < (i + sliding_rate); t++)
			{
				average = average + *(data_in + t);
			}
			//根据不同滑动倍数选择快速除法移位
			switch(sliding_rate)
			{
				case 1://5.333 = 16/3
					*(data + i) = (5.333 * average) >> 4;
				break;
				case 2://3.2 = 16/5
					*(data + i) = (3.2 * average) >> 4;
				break;
				case 3://3.2 = 16/5
					*(data + i) = (4.571 * average) >> 5;
				break;
				default://其他情况直接除法运算 运算极为低效
					*(data_out + i) = average / (2 * sliding_rate + 1);
				break；
			}
		}
	}
}

//有限脉冲响应数字滤波器FIR


//无限脉冲响应数字滤波器IIR
//二阶巴特沃夫低通滤波器参数
static int32_t  fr,ohm,c;
static int32_t 	b0,b1,b2,a1,a2;
static int32_t	data_delay[3];


//无浮点硬件加速，作定点数转化处理
//参考https://zhuanlan.zhihu.com/p/357619650
void Filter_Init(void)
{
	fr = cutoff_freq * 4096 / sample_freq;//整形数据放大4096处理
	//tan()=sin()/cos()  *4096定点数据避免数据丢失
	ohm = SIN_COS_TABLE[fr>>4] / SIN_COS_TABLE[((fr>>4)+128)&0x1ff] * 4096;
	c = 4096 + 1.414 * ohm + ((ohm * ohm)>>12);//c 也是*4096
	
	b0 = ohm * ohm / c;
	b1 = 2 * b0;
	b2 = b0;
	a1 = 2 * 4096 * (((ohm * ohm)>>12) - 4096) / c;
	a2 = 4096 * (4096 - 1.414 * ohm + ((ohm * ohm)>>12)) / c;
}

//int32_t data 输入需要滤波的数据 数据1：4096 需要放大4096倍后输入
int32_t Butterworth_Second(int32_t data)
{
	int32_t output;

	data_delay[0] = (data - data_delay[1] * a1 - data_delay[2] * a2)>>12;
	output = (data_delay[0] * b0 + data_delay[1] * b1 + data_delay[2] * b2)>>12;
	data_delay[2] = data_delay[1];
	data_delay[1] = data_delay[0];
	
	return output;
}


//模拟滤波器IIR
//滑动滤波器

