#include "filter.h"
#include "stdint.h"
#include "sin_cos.h"


//���װ����ַ��ͨ�˲�������
static int32_t  fr,ohm,c;
static int32_t 	b0,b1,b2,a1,a2;
static int32_t	data_delay[3];

//�޸���Ӳ�����٣���������ת������
//�ο�https://zhuanlan.zhihu.com/p/357619650
void Filter_Init(void)
{
	fr = cutoff_freq * 4096 / sample_freq;//�������ݷŴ�4096����
	//tan()=sin()/cos()  *4096�������ݱ������ݶ�ʧ
	ohm = SIN_COS_TABLE[fr>>4] / SIN_COS_TABLE[((fr>>4)+128)&0x1ff] * 4096;
	c = 4096 + 1.414 * ohm + ((ohm * ohm)>>12);//c Ҳ��*4096
	
	b0 = ohm * ohm / c;
	b1 = 2 * b0;
	b2 = b0;
	a1 = 2 * 4096 * (((ohm * ohm)>>12) - 4096) / c;
	a2 = 4096 * (4096 - 1.414 * ohm + ((ohm * ohm)>>12)) / c;
}

//int32_t data ������Ҫ�˲������� ����1��4096 ��Ҫ�Ŵ�4096��������
int32_t Butterworth_Second(int32_t data)
{
	int32_t output;

	data_delay[0] = (data - data_delay[1] * a1 - data_delay[2] * a2)>>12;
	output = (data_delay[0] * b0 + data_delay[1] * b1 + data_delay[2] * b2)>>12;
	data_delay[2] = data_delay[1];
	data_delay[1] = data_delay[0];
	
	return output;
}
