/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "foc.h"
#include "sin_cos.h"
#include "modbus.h"
#include "usart_control.h"
#include "object_commicate.h"
#include "control_loop.h"
#include "hal_my.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t ADC_Data[2];					//ADC采样DMA储存数据地址
FOC_Motor Motor1;							//定义电机结构体Motor1
FOC_PID	FOC_PID_Current_Iq,FOC_PID_Current_Id,FOC_PID_Speed,FOC_PID_Position;			//定义全局PID结构体
uint16_t Loop_Time_Count;


uint8_t Tx_Data[TX_BUFF_LONG];					//定义串口接受和发送缓冲区长度
uint8_t Rx_Data[RX_BUFF_LONG];

uint16_t Tx_Encoder[2] = {0x8300,0x0000};  //定义6813编码器收发数据 burst模式
uint16_t Rx_Encoder[2];

//编码器获取数据查看验证
uint16_t Transfer1[3];
//uint16_t Angle_Transfer[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}
 
//重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{		
	int ch;
	HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	//通讯变量初始化 获取通讯变量个数并初始化控制变量
	Commicate_Data_Init();
	
	//电机参数初始化
	FOC_Motor_Init(&Motor1);
	
	//PID参数初始化
	Current_PID_Init(&FOC_PID_Current_Iq);
	Current_PID_Init(&FOC_PID_Current_Id);
	
	//通讯变量初始化并获取变量数
	Commicate_Data_Init();
	
	//校准ADC采样
	//同步注入采样中需将adc模式设置为连续采样模式才可以使用，仍需对规则组进行部分配置，如规则采样触发，采样数，数据对齐方式
	//同步注入采样中adc1为主采样器，adc2为从配置器，因此触发adc1即可触发adc2
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	
	//唤醒drv8313
	HAL_GPIO_WritePin(Power_Reset_GPIO_Port,Power_Reset_Pin,GPIO_PIN_SET);//拉高drv8313重置引脚
	//点灯
	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);
	//获取偏移角度
	Encoder_Offest(&Motor1);
	//获取两相电流采样修正值
	ADC_Current_Offest(&Motor1);
	//使能drv8313
//	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);	
//	HAL_GPIO_WritePin(Power_Reset_GPIO_Port,Power_Reset_Pin,GPIO_PIN_SET);//拉高drv8313重置引脚

	//定时器2开启1ms中断
	HAL_TIM_Base_Start_IT(&htim2);
	//开启串口DMA发送和接受
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)&Rx_Data,RX_BUFF_LONG);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&Tx_Data,sizeof(Tx_Data));

	//使能串口空闲中断
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	
	//初始状态
	Control_Data.Work_Model = 0;
	Control_Data.Enable = 1;
	Control_Data.Open_Loop_Voltage = 0;
	
	//开启cc4比较通道触发adc注入采样
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			
//		printf("%d,%d\n",Motor1.Ia,Motor1.Ib);  //使用printf打印需要关闭串口1的空闲中断和dma传输，printf重定向串口1就是使用串口1进行简单发送接受
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		HAL_Delay(10);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//ADC注入采样完成回调函数
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc2)
	{
		//当此采样数据用于下次数据甚至下下次数据更新，具体看计算时间，控制在约25us内可以在下次执行，超出则只能在下下次执行
		//角度获取与电流采样同周期
		//获取a,b相电流采样值  反相增益所以加负号  开环给零电压测试
		Motor1.Ia = -(HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1) - Motor1.Ia_Offect);
		Motor1.Ib = -(HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1) - Motor1.Ib_Offect);
		
		//获取电角度
		Get_Electrical_Angle(&Motor1);
		//判断工作状态决定是否使能mos管工作
		if(Control_Data.Enable_Buffer != Control_Data.Enable)
		{
				if(Control_Data.Enable)
				{
					HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);//使能drv8313
				}
				else
				{
					HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);//关闭使能drv8313
					Motor1.Speed_Filter = 0;
					Motor1.Speed_Filter_Loop = 0;//退出使能状态清除速度滤波器值
				}	
				Control_Data.Enable_Buffer = Control_Data.Enable;
		}
		//进行FOC控制
		FOC_Control(&Motor1);
	}
}
//1ms中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		//计算速度 1ms编码器变化数值  区分正负
    //与速度环计算速度方式保持一致  速度分区正负  由运动方向控制
//		Motor1.Encoder_Angle = 16384 - Get_Angle_MT6813(&Motor1);
    //保证读取编码器值为正
    if(Motor1.Encoder_Angle - Motor1.Encoder_Offest < 0)
		  Motor1.Encoder_Angle = Motor1.Encoder_Angle - Motor1.Encoder_Offest + 16384;
	  else
		  Motor1.Encoder_Angle = Motor1.Encoder_Angle - Motor1.Encoder_Offest;
    //本次读取编码器减去前一次得出速度
		Motor1.Encoder_Speed_Angle = Motor1.Encoder_Angle - Motor1.Last_Encoder;
    //运动方向判断速度正负
    if(Control_Data.Work_Direction == 0)//正转速度必为正
    {
      if(Motor1.Encoder_Speed_Angle < 0)
        Motor1.Encoder_Speed_Angle = 16384 + Motor1.Encoder_Speed_Angle;
    }
		else                                //反转速度必为负
    {
      if(Motor1.Encoder_Speed_Angle > 0)
		  	Motor1.Encoder_Speed_Angle = Motor1.Encoder_Speed_Angle - 16384;
    }
    //速度一阶滤波
		Motor1.Speed_Filter = (Motor1.Speed_Filter + 3 * Motor1.Encoder_Speed_Angle)>>2;
    //此次速度作为下一次的过往速度
		Motor1.Last_Encoder = Motor1.Encoder_Angle;
		
		//对外界指令进行处理  1ms处理一次
		//响应编码器与电流重新修正
		if(Control_Data.Encoder_Offect_Process == 1)
		{
			if(Control_Data.Enable_Buffer == 0) //判断此时控制器没有使能才进行校正
			{
				
				Encoder_Offest(&Motor1);//编码器校正
				ADC_Current_Offest(&Motor1);//零电流采样校正
				Control_Data.Encoder_Offect_Process = 0;
			}
		}
		
		
	}	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
