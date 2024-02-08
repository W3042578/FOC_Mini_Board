# FOC_Mini_Board
-----------------------------
4008云台电机FOC控制程序

使用MT6813编码器通过spi通讯获取位置绝对值

uart串口实现modbus RTU通讯

电流环16K，速度环8K，位置环4K

STM32F103C8T6主控，开发环境keil

基本思路：
定时器计数触发ADC采样，ADC采样完成触发回调函数进行foc运算，更新三路mos管占空比计数值
启动程序强拖校准电机对齐alpha轴

ADC注入采样采集双下桥电流
STM32配置定时器1通道4触发adc采样，foc控制函数放在ADC采样完成回调函数
定点数运算+移位+查表，提升程序运行速度


底层具体配置详见STM32CubeMX文件 FOC_Mini_Board.ioc
keil工程启动文件在MDK-ARM文件中
主要文件分为Algorithm、Application、Hardware、Infrastructure四个文件

其中Algorithm包含基本FOC算法、PID控制环和角度正余弦查表

Application包含编码器SPI通讯、串口modbus通讯、滤波、通讯变量字典（usart_control文件和SSD1306未使用）

Hardware包含硬件和底层的参数配置

Infrastructure包含底层文件配置，从CubeMx中底层代码生成后部分修改拷贝过来

------------------------------------------------------------------------------

8010 gimbal motor FOC control program

Using MT6813 encoder to obtain absolute position value through SPI communication

Implementing Modbus RTU communication through UART serial port

Current loop 16K, speed loop 8K, position loop 4K

STM32F103C8T6 main control, development environment keil

Basic idea:
Timer counting triggers ADC sampling, ADC sampling completes triggering callback function for FOC operation, updating the duty cycle count values of three MOSFETs

Start the program to forcefully drag the calibration motor to align with the alpha axis

ADC injection sampling for dual bridge current acquisition

STM32 configuration timer 1, channel 4 triggers ADC sampling, and the FOC control function is placed in the ADC sampling completion callback function

Fixed point number operation+shift+table lookup to improve program running speed

The specific configuration of the underlying layer can be found in the STM32CubeMX file FOC_ Mini_ Board.ioc

The Keil project startup file is in the MDK-ARM file

The main files are divided into four categories: Algorithm, Application, Hardware, and Infrastructure

Among them, Algorithm includes basic FOC algorithm, PID control loop, and angle sine and cosine lookup table

Application includes encoder SPI communication, serial port modbus communication, filtering, and communication variable dictionary (not used in usartself-control file and SSD1306)

Hardware includes hardware and underlying parameter configurations

Infrastructure includes underlying file configurations, which are partially modified and copied from CubeMx after generating the underlying code
