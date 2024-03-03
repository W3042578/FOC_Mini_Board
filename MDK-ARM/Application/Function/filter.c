
#include "filter.h"

#include "table.h"

//数字滤波器FIR
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
