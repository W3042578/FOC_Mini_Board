# FOC_Mini_Board
磁场矢量控制云台电机程序
MT6813  spi通讯获取位置角度
uart串口实现modbus RTU通讯
电流环16K，速度环8K，位置环4K
STM32F103C8T6主控，开发环境keil

基本思路：
定时器计数触发ADC采样，ADC采样完成触发回调函数进行foc运算，更新三路mos管占空比计数值
启动程序强拖校准电机初始角度，给Ud,Uq零电压校准adc采样电流

ADC注入采样采集双下桥电流
STM32配置定时器1通道4触发adc采样，foc控制函数放在ADC采样完成回调函数
定点数运算+移位+查表，提升程序运行速度

后续添加CAN通讯和OLED屏幕
pid自整定、S型曲线、陷波目前还在研究阶段
具体配置详见STM32CubeMX文件
工程启动文件在MDK-ARM文件中

