#include "usart_control.h"
#include "tim.h"
#include "object_commicate.h"

//???????????????????????????16384??????
//????1??trandform_data--????????????
//????2??p--???????????????????
void uart_table_transform(uint16_t transform_data,uint8_t *p)
{
	//												   0	1  2	3  4  5  6  7  8  9
	char transform_table[10] = {48,49,50,51,52,53,54,55,56,57};

	*(p+6) = 10;
	*(p+5) = 13;
	if(transform_data>0)
		*(p+4) = transform_table[transform_data%10];
	else
		*(p+4) = transform_table[0];
	if(transform_data>10)
		*(p+3) = transform_table[(transform_data/10)%10];
	else
		*(p+3) = transform_table[0];
	if(transform_data>100)
		*(p+2) = transform_table[(transform_data/100)%10];
	else
		*(p+2) = transform_table[0];
	if(transform_data>1000)
		*(p+1) = transform_table[(transform_data/1000)%10];
	else
		*(p+1) = transform_table[0];
	if(transform_data>10000)
		*p = transform_table[(transform_data/10000)%10];
	else
		*p = transform_table[0];
}


//???????????????????§á???
void Usart_Direct_Control(uint8_t * data)
{
		
		switch(*data)
			{
				case 0x01://????????
					Control_Data.Enable = *(data + 1);
					if(Control_Data.Enable)
					{
						//????cc4??????????adc??????
						HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
						HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
						HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
						HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
					}
				break;
				case 0x02://?????????
					Control_Data.Work_Direction = *(data + 1);
				break;
				case 0x03://????Uq???
					if(*(data + 1)>11)
						Control_Data.Open_Loop_Voltage = 11;
					else
						Control_Data.Open_Loop_Voltage = *(data + 1);
				break;
				case 0x04:
					
				break;
				default:
					
				break;
			}
}
