#ifndef __FILTER_H
#define __FILTER_H

#include "stdint.h"
//定义滤波的截至频率和采样频率
#define sample_freq		2
#define	cutoff_freq		3


int32_t Butterworth_Second(int32_t data);


#endif
