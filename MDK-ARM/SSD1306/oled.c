#include "oled.h"
#include "main.h"
#include "object_commicate.h"
#include "i2c.h"

uint8_t OLED_GRAM[128][8];
//SSD1306参考代码：https://blog.csdn.net/weixin_42880082/article/details/130180060
//SSD1306具体操作指令详解 https://blog.csdn.net/weixin_44457994/article/details/121484335

//uint8_t OLED_Control(void)
//{
//    
//    

//}
////SSD1306初始化
//void OLED_Init(void)
//{
//    HAL_I2C_Master_Transmit_DMA(&hi2c1,Slave_Address,);
//}


