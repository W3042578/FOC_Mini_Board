/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "stdio.h"
#include "string.h"
#include "modbus.h"
#include "foc.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "parameter.h"
/* USER CODE BEGIN 0 */
//串口宏定义
#define   USART_OSC_START         BIT0
#define   USART_OSC_ERROR         BIT1
#define   USART_WAIT_SEND         BIT2
#define   USART_OSCILL0_BUFFER    16384
#define   SEND_NUMBER             6
//全局定义
static void oscilloscope_send(void);            //串口示波器数据发送
// static    uint32_t  *Channel_map[8];            //通道映射
// static    uint8_t   frame_id;                   //帧ID 测试数据用            
static    uint8_t   uart_status_byte;           //串口示波器状态控制
static    uint8_t   frame_length, Test_Uart;    //Test_Uart:串口示波器发送数据模式 
static    int8_t    buffer_turn_differ;         //缓冲区读取和写入回零圈数差
static    uint16_t  write_address, read_address;
union     _Uart_Debug_Buffer  Debug_Buffer[4];  //单帧数据 双通道
uint8_t   Tx_Buffer[20];
uint8_t   Uart_Test_Buffer[USART_OSCILL0_BUFFER];
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1500000;
//  huart1.Init.BaudRate = 115200;//921600
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_USART1_ENABLE();

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

//空闲中断处理函数
void uart_idleback(UART_HandleTypeDef *huart)
{
	uint8_t remain; 						//定义变量储存DMA剩余传输位
	//停止本次DMA传输
	HAL_UART_DMAStop(&huart1);
	
	//获取DMA未接受到的数据位
	remain = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
	//接受数据的长度等于缓冲区总长减去剩余长度
	Modbus_Length_In = RX_BUFF_LONG - remain;
	
	//将接受到的数组复制到缓冲区  memcpy函数在string.h头文件下
	memcpy(Modbus_Buffer,Rx_Data,Modbus_Length_In);	
	
	//清空接受数组内容  memset函数在string.h头文件下
	memset(Rx_Data,0,Modbus_Length_In);
	
  #ifdef MODBUS_UART
	//Modbus协议处理
	if(Modbus_Process(Modbus_Buffer,Modbus_Length_In,Tx_Data,&Modbus_Length_Out) == 1)//判断Modbus进程是否正常
	{
		//根据处理结果发送数据给主机
		HAL_UART_Transmit_DMA(&huart1,Tx_Data,Modbus_Length_Out); //5位分别为 ID位 功能码  字节数位  高低两位校验码
	}
  #else
  _NEGA(&uart_status_byte, USART_OSC_START);//取反调试状态位
  Test_Uart = Modbus_Buffer[0];
  #endif
	//数据处理结束后重新开始DMA传输
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)&Rx_Data,RX_BUFF_LONG);
}

//串口缓冲区写入发送
void Uart_Loop_Debug_Write(void)
{
  static    uint8_t   to_init,interval;    //串口缓冲初始化,采样周期间隔
  uint8_t   temp;   //中间转换变量
  uint8_t   size;   //采集通道数
  uint32_t  data_transfer[5]; //过渡数组
  if(_TEST(&uart_status_byte, USART_OSC_START))
  {        
    if(Test_Uart <= 8)
    {           
      if(interval == 0)
      {
        switch (Test_Uart)//获取数据
        {
          case 0:
            Debug_Buffer[0].flo = (float)Motor1.Ta;
            Debug_Buffer[1].flo = (float)Motor1.Tb;
            Debug_Buffer[2].flo = (float)Motor1.Tc;
            size = 3;
            interval = 4;
            break;
          case 1:
            Debug_Buffer[0].flo = (float)Motor1.Sin_Angle;
            Debug_Buffer[1].flo = (float)Motor1.Cos_Angle;
            size = 2;
            interval = 2;
            break;
          case 2:
            Debug_Buffer[0].flo = (float)Motor1.Ualph;
            Debug_Buffer[1].flo = (float)Motor1.Ubeta;
            size = 2;
            interval = 2;
            break;
          case 4:
            Debug_Buffer[0].flo = (float)Motor1.Mechanical_Angle;
            size = 1;
            interval = 1;
          break;
          default:
          break;
        }
        //组帧
        temp = size << 2;
        memcpy(data_transfer, &Debug_Buffer[0].un32, temp);  //单帧采集数据放入   
        data_transfer[size] = 0x7f800000; //单帧帧尾结束标志
        // Tx_Buffer[0] = frame_id;          //帧ID写入 帧ID
        // frame_id ++;     //帧ID
        temp = (size + 1) << 2;     //单帧采集数据 + 帧尾标志 byte数
        // memcpy(&Tx_Buffer[1], data_transfer, temp); //从帧头后开始放入采集数据和帧尾结束标志 帧ID
        memcpy(&Tx_Buffer[0], data_transfer, temp); //从帧头后开始放入采集数据和帧尾结束标志
        // frame_length = temp + 1;    //帧ID一个Byte 帧长 = 帧ID + 帧数据 + 帧尾标志 帧ID
        frame_length = temp;
        //帧数据进入缓冲队列
        if((write_address + frame_length) >= USART_OSCILL0_BUFFER)  //帧地址超出回零
        {
          buffer_turn_differ ++;     //写入缓冲回环计数
          write_address = 0;
        }
        //判断写入停止条件
        //1.写入和读出仍在同一环中 或者
        //2.写入超前读出一环，但写入地址还未追上读出地址
        temp = !_TEST(&uart_status_byte, USART_OSC_ERROR);
        if((buffer_turn_differ == 0 || \
        (buffer_turn_differ == 1 && (write_address + frame_length) < read_address)) \
        && temp)
        {
          memcpy(&Uart_Test_Buffer[write_address], Tx_Buffer, frame_length);
          write_address += frame_length;      //帧地址累加计算
        }
        else
        {
          _SET(&uart_status_byte, USART_OSC_ERROR);
        }
        //第一次进入发送，后续由发送完成中断进行处理       
        if((to_init) && (write_address > (frame_length * SEND_NUMBER)))
        {        
          to_init = 0;  
          memcpy(Tx_Data, Uart_Test_Buffer, frame_length * SEND_NUMBER); //取缓冲队列第一帧数据
          HAL_UART_Transmit_DMA(&huart1, Tx_Data, frame_length  << 2);//单帧数据发送
        }       
        //读出等待触发，等待再次采集到数据后继续发送  继续发送两个执行条件:
        //1.写入与读出同环，写入超前读出
        //2.写入超前读出一环，写入地址大于0起始指定帧数数据大小
        if(_TEST(&uart_status_byte, USART_WAIT_SEND))
          if(((read_address + (frame_length * SEND_NUMBER) < write_address) && buffer_turn_differ == 0) || \
          (((frame_length * SEND_NUMBER) < write_address) && buffer_turn_differ == 1)) 
          {
            _CLEAN(&uart_status_byte, USART_WAIT_SEND);
            oscilloscope_send();         
          } 
      }		
      interval --;  
    }
    else//保存数据在ram，存满再发送
    {
      Debug_Buffer[0].flo = (float)Motor1.Mechanical_Angle;
      if(to_init)
      {
        to_init = 0;
        //  HAL_UART_Transmit_DMA(&huart1, uart_test_buffer, 2048);
        _CLEAN(&uart_status_byte, USART_OSC_START); //结束发送
      }
    } 
  }
  else
  {
    memset(Uart_Test_Buffer, 0, sizeof(Uart_Test_Buffer));
    read_address = 0;
    write_address = 0;
	  buffer_turn_differ = 0;
	  _CLEAN(&uart_status_byte, USART_OSC_ERROR | USART_WAIT_SEND);
    to_init = 1;
  }
}

//串口发送完成回调 由dma传输完成置位串口发送完成标志位触发
//完成发送后判断是否继续发送
void HAL_UART_Tx_End_Callback(UART_HandleTypeDef *huart)
{
  if(Test_Uart <= 8)
  {    
    if(buffer_turn_differ == 0 && (read_address + (frame_length * SEND_NUMBER) >= write_address))
    {
      //串口发送读取过快，停止直接发送。置位在数据更新后再发送
      _SET(&uart_status_byte, USART_WAIT_SEND);
    }
    else if(_TEST(&uart_status_byte, USART_OSC_ERROR))   //出现错误停止不再继续发送
    {
      
    }
    else
    {
      oscilloscope_send();
    }
   
  }
  else
  {
    _SET(&uart_status_byte, USART_OSC_ERROR);
  }
}

//串口示波器数据发送
static void oscilloscope_send(void)
{
  if((read_address + (frame_length * SEND_NUMBER)) >= USART_OSCILL0_BUFFER)  //读取地址限制
  {
    buffer_turn_differ --;     //读出缓冲回环计数
    read_address = 0;        
  }
  memcpy(Tx_Data, &Uart_Test_Buffer[read_address], frame_length * SEND_NUMBER); //取缓冲队列数据
  HAL_UART_Transmit_DMA(&huart1, Tx_Data, frame_length * SEND_NUMBER);          //单帧数据发送
  read_address += (frame_length * SEND_NUMBER);   //读取地址累加
}

/* USER CODE END 1 */
