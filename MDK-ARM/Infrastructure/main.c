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
#include "encoder.h"
#include "parameter.h"
#include "basic_function.h"
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



//环路设计中， 
//环路间比例最小为2，保证控制环路稳定


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
	
	//校准ADC采样
	//同步注入采样中需将adc模式设置为连续采样模式才可以使用，仍需对规则组进行部分配置，如规则采样触发，采样数，数据对齐方式
	//同步注入采样中adc1为主采样器，adc2为从配置器，因此触发adc1即可触发adc2
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);

	//开启规则组常规采样
	HAL_ADC_Start(&hadc2); 
	HAL_ADC_Start(&hadc1);

  	//开启注入组采样，注意注入采样控制中断的处理函数中会关闭注入中断使能位
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);

 	 //同步dual模式为多模式采样，同步规则采样需要开启DMA，注入采样建立在同步采样基础上
	HAL_ADCEx_MultiModeStart_DMA(&hadc1,ADC_Data,2);
	
	//开启cc4比较通道触发adc注入采样
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

	//定时器2开启1ms中断
	HAL_TIM_Base_Start_IT(&htim2);
	//开启串口DMA发送和接受
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)&Rx_Data,RX_BUFF_LONG);

	//使能串口空闲中断
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	
	//参数初始化
	Parameter_Init();
	
	//获取两相电流采样修正值
	ADC_Current_Offest(&Motor1);

	//点灯
	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
	
	// HAL_GPIO_WritePin(Test1_GPIO_Port,Test1_Pin,GPIO_PIN_SET);//环路执行周期测试

    	
    //编码器方向判断
//    if(Work_Status.bits.Direction_Encoder == 1 && Work_Status.bits.Offest_Encoder == 0)//方向判断在零位校正后
//    {
//      //对前进后角度编码器值累加求平均
//      //前后编码器值对比判断方向是否相同
//      encoder1.Encoder_Direction_Position = encoder1.Encoder_Direction_Position + encoder1.Encoder_Angle;
//      Number_Encoder_Direction = Number_Encoder_Direction - 1;
//      if(Number_Encoder_Direction < 0)
//      {
//        encoder1.Encoder_Direction_Position = encoder1.Encoder_Direction_Position >> 5;
//        Work_Status.bits.Direction_Encoder = 0;
//        if (encoder1.Encoder_Direction_Position < encoder1.Encoder_Offest_Data)
//        {
//          if(encoder1.Encoder_Offest_Data - encoder1.Encoder_Direction_Position < 32768)
//            encoder1.Encoder_Direction = 1;//反向
//          else
//            encoder1.Encoder_Direction = 0;//同向
//        }
//        else
//        {
//          if(encoder1.Encoder_Direction_Position - encoder1.Encoder_Offest_Data < 32768)
//            encoder1.Encoder_Direction = 0;
//          else
//            encoder1.Encoder_Direction = 1;
//        }
//      }
//      Motor1.Direction = encoder1.Encoder_Direction;//多值赋予方便多电机设置
//    }

	//获取a,b相电流采样值  开环给零电压测试 离开电机方向为负
	Motor1.Ia = -(HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1) - Motor1.Ia_Offect);
	Motor1.Ib = -(HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1) - Motor1.Ib_Offect);
	

	//PWM使能控制
	Enable_Logic_Control();
	
	//进行FOC控制
	FOC_Control(&Motor1);
	
	//STM32 HAL 三相PWM比较值设置
	STM32_HAL_PWM_SET_Compare(&Motor1);
	
	//初始角校准,累加求和
	Get_Initial_Angle_Offest(&Motor1);
	
	//在同步注入中断回调中hal库默认关闭该中断使能，因此在执行完注入中断后再次打开中断使能
	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
	
	// HAL_GPIO_WritePin(Test1_GPIO_Port,Test1_Pin,GPIO_PIN_RESET); //环路执行周期测试

}
//定时器中断回调函数 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		//1ms中断回调函数
		Interrupt_1MS();
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
