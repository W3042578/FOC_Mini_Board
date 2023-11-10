#include "main.h"
#include "modbus.h"
#include "string.h"
#include "object_commicate.h"
#include "usart_control.h"
//查表方式实现Modbus校验
static const uint8_t Modbus_CRC_Hig[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40
};

static const uint8_t Modbus_CRC_Low[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
0x41, 0x81, 0x80, 0x40
};

uint8_t Modbus_Length_In;								//接受到的一帧Modbus数据长度，有多少个byte
MODBUS Modbus;													//Modbus数据处理结构体
uint8_t Modbus_Buffer[30];							//Modbus发送缓冲区长度
uint16_t	Modbus_Length_Out;

//接受不定长数据：串口接受数据，一位时间内没有接受到数据触发空闲中断，空闲中断对DMA获取数据进行处理
//输入参数:接受到的串口数据指针:data_in		串口数据长度:length_in		输出发送数据指针:data_out   
//读取10个数据时Tx_Data数组大小应不小于25个byte
uint8_t Modbus_Process(uint8_t *data_in,uint8_t length_in,uint8_t *data_out,uint16_t* length_out)
{
	//中间变量
	commicate_code transfer;					//存储需要读取的数据
	uint16_t read_limit;							//判断读取数据允许最大长度，避免读取数据地址超出
	uint16_t write_limit;							//判断写入数据允许最大长度，避免写入数据地址超出
	
	//Modbus结构体数据清零
	memset(&Modbus,0,sizeof(Modbus));
	
	
	//校验数据传输
	Modbus.CRC_Self_Data.ALL = Modbus_CRC_Data(data_in,length_in - 2);
	Modbus.CRC_RX_Data.ALL =  (*(data_in + length_in - 2)<<8)|*(data_in + length_in - 1); //取Modbus_Buffer最后两位处理
																								

	if(Modbus.CRC_Self_Data.ALL == Modbus.CRC_RX_Data.ALL)//校验正确
	{
		Modbus.ID = *data_in;				//Modbus_Buffer[0]
		if(Modbus.ID == Slave_ID)		//判断从机ID是否对应
		{
				Modbus.Function_Code = *(data_in + 1);															//Modbus_Buffer[1]     获取传输数据功能码
				Modbus.Start_Address = (*(data_in + 2)<<8) | *(data_in + 3);				//Modbus_Buffer[2]和Modbus_Buffer[3]处理   获取传输数据开始地址，依次判断要改变的变量索引

				for(uint8_t i = 0; i<=Commicate_Data_All_Number; i++)								//遍历限制范围为object_commitcate.c文件中结构体数组数量
					{
						if(Commicate_Code[i].Index == Modbus.Start_Address)							//Modbus数据起始地址判断对应变量索引位置
						{
							Modbus.Const_Columns_Number = i;													
							break;
						}
						if(i == Commicate_Data_All_Number )
						{
							Modbus.Error = 1;
							return 0;							//没有找到对应数据索引，直接返回
						}
					}																											
				switch(Modbus.Function_Code)//根据功能码执行
				{
					// 主机：0x03
					//  01   03     00  01       00  01       D5 CA
					//  ID  功能码  起始地址   读取寄存器个数    校验码
					// 从机返回：
					//	01   03       02         00  03       F8 453
					//  ID  功能码   字节数    返回数据内容      校验码
					case 0x03:  //读取多个寄存器数据
							Modbus.Register_Number = (*(data_in + 4) << 8) | *(data_in + 5);	//Modbus_Buffer[4]和Modbus_Buffer[5]处理  获取寄存器数值
							Modbus.Byte_Number = Modbus.Register_Number * 2;									//Modbus中 字节数 = 寄存器 x2  
							
							//判断数据读取有无超出范围
							if(Modbus.Register_Number + Modbus.Const_Columns_Number > Commicate_Data_All_Number)//寻得读取数据起始加上要读取数量超出变量数组个数
								read_limit = Modbus.Register_Number - (Modbus.Register_Number + Modbus.Const_Columns_Number - Commicate_Data_All_Number);	//获取允许读取的数据量
							else
								read_limit = Modbus.Register_Number;//没有超出不作限制
							
							//对没有超出数据一一进行读取
							for(uint8_t i = 0; i<read_limit; i++)
							{
								transfer = Commicate_Code[Modbus.Const_Columns_Number + i];
								if(transfer.Data_Type == 2)
								{
									Modbus.Read_Data_Array[i].ALL = *((uint16_t*)transfer.Commicate_Pointor);					//依次获取变量地址对应数据  获取将要读取的变量值  void指针可以赋值给其他数据类型，但需要对指针进行强制转换
									Modbus.Read_To_Send[2*i] = Modbus.Read_Data_Array[i].byte.Transfer_Data_8bit_low;//i指16位数组中数据位置，2*i指对应8位数组中数据对应位置
									Modbus.Read_To_Send[2*i+1] = Modbus.Read_Data_Array[i].byte.Transfer_Data_8bit_high;
								}
								else if(transfer.Data_Type ==1)
								{
									Modbus.Read_To_Send[2*i+1] = *((uint8_t*)transfer.Commicate_Pointor);
								}					
							}
		
							//准备数据发送
							*data_out = Modbus.ID;
							*(data_out + 1) = Modbus.Function_Code;
							*(data_out + 2) = Modbus.Byte_Number;
							memcpy(data_out+3, Modbus.Read_To_Send, Modbus.Byte_Number);							//填入要读取数据，8位数组,实际读取数据长度不够对其他位填0
							Modbus.CRC_TX_Data.ALL = Modbus_CRC_Data(data_out,3 + Modbus.Byte_Number);//计算将要发送数据校验码，数据存放低位开始，多余空位放在高位
							*(data_out + 3 + Modbus.Byte_Number) = Modbus.CRC_TX_Data.nchar.high;			//填入高8位校验码
							*(data_out + 4 + Modbus.Byte_Number) = Modbus.CRC_TX_Data.nchar.low;			//填入低8位校验码
							*length_out = Modbus.Byte_Number + 5;
						return 1;
					// 主机：0x06
					//  01   06     00  01       00  02       59 C8
					//  ID  功能码  写入地址		写入数据内容    	校验码
					// 从机返回：
					//	01   06     00  01       00  02       59 C8
					//  ID  功能码  写入地址  	写入数据内容    	校验码	
					case 0x06:  //写入一个寄存器数据
						transfer = Commicate_Code[Modbus.Const_Columns_Number];
						Modbus.Write_Data = (*(data_in + 4) << 8) | *(data_in + 5);//获取要写入数据内容
							if(transfer.Data_Type == 1)
							{
								if(Modbus.Write_Data > 0xFF)//内部设置数据类型为一个字节，实际输入超出1个字节byte范围直接返回0，输入超出错误		
								{
									Modbus.Error = 1;
									return 0;	
								}	
								*((uint8_t*)(transfer.Commicate_Pointor)) = Modbus.Write_Data;//将输入数据赋值给对应变量
							}
							else if(transfer.Data_Type == 2)
							{
								*((uint16_t*)(transfer.Commicate_Pointor)) = Modbus.Write_Data;//将输入数据赋值给对应变量
							}
							 //变量数据大小写入错误  将8位数据转为16位指针错误
							
							//准备数据发送  写入数据和开始地址均为16位数据，data_out为8位数组串口发送
							*data_out = Modbus.ID;
							*(data_out + 1) = Modbus.Function_Code;
							*(data_out + 2) = Modbus.Start_Address >>8 ;					//取开始地址高两位
							*(data_out + 3) = Modbus.Start_Address & 0xFF;				//取开始地址低两位
							*(data_out + 4) = Modbus.Write_Data >>8;							//取写入数据高两位
							*(data_out + 5) = Modbus.Write_Data & 0xFF;						//取写入数据低两位
							Modbus.CRC_TX_Data.ALL = Modbus_CRC_Data(data_out,6);	//计算将要发送数据校验码，数据存放低位开始，多余空位放在高位
							*(data_out + 6)	= Modbus.CRC_TX_Data.nchar.high;			//填入高8位校验码
							*(data_out + 7) = Modbus.CRC_TX_Data.nchar.low;				//填入低8位校验码
							*length_out = 8;
						return 1;
					// 主机：0x10
					//  01   10    	00  01       00  06      		0C					00 03 	00 00	 00 00	 00 00 	00 00 	00 05						04 00
					//  ID  功能码  写入地址		写入寄存器个数		写入字节数    											写入内容														校验码
					// 从机返回：
					//	01   10     00  01       00  06       59 C8
					//  ID  功能码  写入地址  	写入寄存器个数   	校验码				
					case 0x10:  //写入多个寄存器数据
							Modbus.Register_Number = (*(data_in + 4) << 8) | *(data_in + 5);
							Modbus.Byte_Number = *(data_in + 6);
							
							//判断数据写入有无超出范围
							if(Modbus.Register_Number + Modbus.Const_Columns_Number > Commicate_Data_All_Number)//寻得写入数据起始加上要写入数量超出变量数组个数
								{
									write_limit = Modbus.Register_Number - (Modbus.Register_Number + Modbus.Const_Columns_Number - Commicate_Data_All_Number);	//获取允许写入的数据量
								}
							else
								write_limit = Modbus.Register_Number;																			//没有超出不作限制
							
							memcpy(Modbus.Write_Data_Array,data_in + 7,write_limit * 2);								//按照写入限制
							//对没有超出数据一一进行写入
							for(uint8_t i = 0; i<write_limit; i++)
							{
								transfer = Commicate_Code[Modbus.Const_Columns_Number + i];
								*((uint16_t*)transfer.Commicate_Pointor) = (Modbus.Write_Data_Array[i].byte.Transfer_Data_8bit_high) << 8 | Modbus.Write_Data_Array[i].byte.Transfer_Data_8bit_low;			
							}
					
							//准备数据发送
							*data_out = Modbus.ID;
							*(data_out + 1) = Modbus.Function_Code;
							*(data_out + 2) = Modbus.Start_Address >>8 ;					//取开始地址高两位
							*(data_out + 3) = Modbus.Start_Address & 0xFF;				//取开始地址低两位
							*(data_out + 4) = Modbus.Register_Number >>8;					//取写入寄存器个数高两位
							*(data_out + 5) = Modbus.Register_Number & 0xFF;			//取写入寄存器个数低两位
							Modbus.CRC_TX_Data.ALL = Modbus_CRC_Data(data_out,6);	//计算将要发送数据校验码，数据存放低位开始，多余空位放在高位
							*(data_out + 6)	= Modbus.CRC_TX_Data.nchar.high;			//填入高8位校验码
							*(data_out + 7) = Modbus.CRC_TX_Data.nchar.low;				//填入低8位校验码
							*length_out = 8;
						return 1;
					default:
						Error_Message.bits.Modbus_Status = 1;
						return 0;//ID正确、校验正确情况下没有找到对应功能码
				}
		}
		else
		{
			Error_Message.bits.Modbus_Status = 1;
			return 0;				//校验正确但ID不正确
		}
	}
	else
	{
		Error_Message.bits.Modbus_Status = 1;
		return 0;					//校验不正确
	}
}
//Modbus协议校验查表实现
uint16_t Modbus_CRC_Data(uint8_t *data,uint16_t length)
{
	uint8_t CRCHig = 0xFF;
  uint8_t CRCLow = 0xFF;
    int iIndex;

	while(length--)
	{
			iIndex = CRCLow ^*( data++ );
			CRCLow = ( uint8_t )( CRCHig ^Modbus_CRC_Hig[iIndex] );
			CRCHig = Modbus_CRC_Low[iIndex];
	}
    return ( uint16_t )(  CRCLow << 8 | CRCHig );
}



