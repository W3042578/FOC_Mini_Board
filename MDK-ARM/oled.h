#ifndef __OLED_H
#define __OLED_H
#include "stdint.h"

//模拟OLED内存
extern uint8_t OLED_GRAM[128][8];


#define     Slave_Address       0X78        //从机地址
#define     OLED_Mode           0           //OLED模式
#define     X_Width             128         //x轴宽度
#define     Y_Width             64          //y轴宽度
#define     Brightness          0XFF        //最大亮度
#define     Max_Bow             64          //最大行数
#define     Max_Bolumn          128         //最大列数




#endif


