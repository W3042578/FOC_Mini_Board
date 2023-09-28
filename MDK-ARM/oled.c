#include "oled.h"
#include "main.h"
#include "object_commicate.h"
#include "i2c.h"

uint8_t OLED_GRAM[128][8];//模拟SSD1306 OLED驱动芯片 参考：https://blog.csdn.net/weixin_43872149/article/details/104712248

//SSD1306 OLED驱动芯片IIC通讯显示屏幕
uint8_t OLED_Control(void)
{
    
    

}
//OLED屏幕初始化
void OLED_Init(void)
{
    HAL_I2C_Master_Transmit_DMA(&hi2c1,Slave_Address,);
}


