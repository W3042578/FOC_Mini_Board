#include "oled.h"
#include "main.h"
#include "object_commicate.h"
#include "i2c.h"

uint8_t OLED_GRAM[128][8];//ģ��SSD1306 OLED����оƬ �ο���https://blog.csdn.net/weixin_43872149/article/details/104712248

//SSD1306 OLED����оƬIICͨѶ��ʾ��Ļ
uint8_t OLED_Control(void)
{
    
    

}
//OLED��Ļ��ʼ��
void OLED_Init(void)
{
    HAL_I2C_Master_Transmit_DMA(&hi2c1,Slave_Address,);
}


