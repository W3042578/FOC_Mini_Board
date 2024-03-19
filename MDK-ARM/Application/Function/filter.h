#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f1xx_hal.h"

//定义滤波的截至频率和采样频率
#define sample_freq		2
#define	cutoff_freq		3

//滑动滤波器
void Sliding_Filter(int16 * data_in,int16 * data_out,uint8 sliding_rate);

//巴特沃斯低通滤波器
int32_t Butterworth_Second(int32_t data);


#endif
