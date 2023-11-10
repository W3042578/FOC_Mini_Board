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
uint32_t ADC_Data[2];					//ADC����DMA�������ݵ�ַ
FOC_Motor Motor1;							//�������ṹ��Motor1
FOC_PID	FOC_PID_Current_Iq,FOC_PID_Current_Id,FOC_PID_Speed,FOC_PID_Position;			//����ȫ��PID�ṹ��
uint16_t Loop_Time_Count;


uint8_t Tx_Data[TX_BUFF_LONG];					//���崮�ڽ��ܺͷ��ͻ���������
uint8_t Rx_Data[RX_BUFF_LONG];

uint16_t Tx_Encoder[2] = {0x8300,0x0000};  //����6813�������շ����� burstģʽ
uint16_t Rx_Encoder[2];

//��������ȡ���ݲ鿴��֤
uint16_t Transfer1[3];
//uint16_t Angle_Transfer[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//�ض���c�⺯��printf������DEBUG_USART���ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
	/* ����һ���ֽ����ݵ�����DEBUG_USART */
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}
 
//�ض���c�⺯��scanf������DEBUG_USART����д����ʹ��scanf��getchar�Ⱥ���
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
	
	//ͨѶ������ʼ�� ��ȡͨѶ������������ʼ�����Ʊ���
	Commicate_Data_Init();
	
	//���������ʼ��
	FOC_Motor_Init(&Motor1);
	
	//PID������ʼ��
	Current_PID_Init(&FOC_PID_Current_Iq);
	Current_PID_Init(&FOC_PID_Current_Id);
	
	//ͨѶ������ʼ������ȡ������
	Commicate_Data_Init();
	
	//У׼ADC����
	//ͬ��ע��������轫adcģʽ����Ϊ��������ģʽ�ſ���ʹ�ã�����Թ�������в������ã��������������������������ݶ��뷽ʽ
	//ͬ��ע�������adc1Ϊ����������adc2Ϊ������������˴���adc1���ɴ���adc2
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	
	//����drv8313
	HAL_GPIO_WritePin(Power_Reset_GPIO_Port,Power_Reset_Pin,GPIO_PIN_SET);//����drv8313��������
	//���
	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);
	//��ȡƫ�ƽǶ�
	Encoder_Offest(&Motor1);
	//��ȡ���������������ֵ
	ADC_Current_Offest(&Motor1);
	//ʹ��drv8313
//	HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);	
//	HAL_GPIO_WritePin(Power_Reset_GPIO_Port,Power_Reset_Pin,GPIO_PIN_SET);//����drv8313��������

	//��ʱ��2����1ms�ж�
	HAL_TIM_Base_Start_IT(&htim2);
	//��������DMA���ͺͽ���
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)&Rx_Data,RX_BUFF_LONG);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&Tx_Data,sizeof(Tx_Data));

	//ʹ�ܴ��ڿ����ж�
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	
	//��ʼ״̬
	Control_Data.Work_Model = 0;
	Control_Data.Enable = 1;
	Control_Data.Open_Loop_Voltage = 0;
	
	//����cc4�Ƚ�ͨ������adcע�����
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			
//		printf("%d,%d\n",Motor1.Ia,Motor1.Ib);  //ʹ��printf��ӡ��Ҫ�رմ���1�Ŀ����жϺ�dma���䣬printf�ض��򴮿�1����ʹ�ô���1���м򵥷��ͽ���
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

//ADCע�������ɻص�����
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc2)
	{
		//���˲������������´������������´����ݸ��£����忴����ʱ�䣬������Լ25us�ڿ������´�ִ�У�������ֻ�������´�ִ��
		//�ǶȻ�ȡ���������ͬ����
		//��ȡa,b���������ֵ  �����������ԼӸ���  ���������ѹ����
		Motor1.Ia = -(HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1) - Motor1.Ia_Offect);
		Motor1.Ib = -(HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1) - Motor1.Ib_Offect);
		
		//��ȡ��Ƕ�
		Get_Electrical_Angle(&Motor1);
		//�жϹ���״̬�����Ƿ�ʹ��mos�ܹ���
		if(Control_Data.Enable_Buffer != Control_Data.Enable)
		{
				if(Control_Data.Enable)
				{
					HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_SET);//ʹ��drv8313
				}
				else
				{
					HAL_GPIO_WritePin(PWM_EN_GPIO_Port,PWM_EN_Pin,GPIO_PIN_RESET);//�ر�ʹ��drv8313
					Motor1.Speed_Filter = 0;
					Motor1.Speed_Filter_Loop = 0;//�˳�ʹ��״̬����ٶ��˲���ֵ
				}	
				Control_Data.Enable_Buffer = Control_Data.Enable;
		}
		//����FOC����
		FOC_Control(&Motor1);
	}
}
//1ms�жϻص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		//�����ٶ� 1ms�������仯��ֵ  ��������
    //���ٶȻ������ٶȷ�ʽ����һ��  �ٶȷ�������  ���˶��������
//		Motor1.Encoder_Angle = 16384 - Get_Angle_MT6813(&Motor1);
    //��֤��ȡ������ֵΪ��
    if(Motor1.Encoder_Angle - Motor1.Encoder_Offest < 0)
		  Motor1.Encoder_Angle = Motor1.Encoder_Angle - Motor1.Encoder_Offest + 16384;
	  else
		  Motor1.Encoder_Angle = Motor1.Encoder_Angle - Motor1.Encoder_Offest;
    //���ζ�ȡ��������ȥǰһ�εó��ٶ�
		Motor1.Encoder_Speed_Angle = Motor1.Encoder_Angle - Motor1.Last_Encoder;
    //�˶������ж��ٶ�����
    if(Control_Data.Work_Direction == 0)//��ת�ٶȱ�Ϊ��
    {
      if(Motor1.Encoder_Speed_Angle < 0)
        Motor1.Encoder_Speed_Angle = 16384 + Motor1.Encoder_Speed_Angle;
    }
		else                                //��ת�ٶȱ�Ϊ��
    {
      if(Motor1.Encoder_Speed_Angle > 0)
		  	Motor1.Encoder_Speed_Angle = Motor1.Encoder_Speed_Angle - 16384;
    }
    //�ٶ�һ���˲�
		Motor1.Speed_Filter = (Motor1.Speed_Filter + 3 * Motor1.Encoder_Speed_Angle)>>2;
    //�˴��ٶ���Ϊ��һ�εĹ����ٶ�
		Motor1.Last_Encoder = Motor1.Encoder_Angle;
		
		//�����ָ����д���  1ms����һ��
		//��Ӧ�������������������
		if(Control_Data.Encoder_Offect_Process == 1)
		{
			if(Control_Data.Enable_Buffer == 0) //�жϴ�ʱ������û��ʹ�ܲŽ���У��
			{
				
				Encoder_Offest(&Motor1);//������У��
				ADC_Current_Offest(&Motor1);//���������У��
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
