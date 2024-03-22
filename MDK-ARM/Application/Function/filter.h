#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f1xx_hal.h"

//定义滤波的截至频率和采样频率
#define sample_freq		2
#define	cutoff_freq		3

//滑动滤波器
//data_in：输入数据指针	data_out：输出数据指针	data_count：数据个数	sliding_rate：滑动倍数
void Sliding_Filter(int16_t * data_in, int16_t * data_out, int16_t data_count, uint8_t sliding_rate)

//巴特沃斯低通滤波器
int32_t Butterworth_Second(int32_t data);


#endif
