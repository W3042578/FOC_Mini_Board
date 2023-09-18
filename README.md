# FOC_Mini_Board
磁场矢量控制云台电机程序，使用MT6813获取位置角度，使用uart串口实现modbus RTU通讯，电流环16K，速度环8K，位置环4K，STM32F103C8T6主控，开发环境keil

ADC注入采样采集双下桥电流
STM32配置定时器1通道4触发adc采样，foc控制函数放在ADC采样完成回调函数

工程启动文件在MDK-ARM文件中

