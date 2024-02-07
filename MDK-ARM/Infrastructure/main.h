/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Test2_Pin GPIO_PIN_1
#define Test2_GPIO_Port GPIOA
#define Fault_Check_Pin GPIO_PIN_2
#define Fault_Check_GPIO_Port GPIOA
#define Test1_Pin GPIO_PIN_3
#define Test1_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define U_ADC_Pin GPIO_PIN_0
#define U_ADC_GPIO_Port GPIOB
#define V_ADC_Pin GPIO_PIN_1
#define V_ADC_GPIO_Port GPIOB
#define LED_RUN_Pin GPIO_PIN_12
#define LED_RUN_GPIO_Port GPIOB
#define LED_ERR_Pin GPIO_PIN_13
#define LED_ERR_GPIO_Port GPIOB
#define LED_Status_Pin GPIO_PIN_14
#define LED_Status_GPIO_Port GPIOB
#define PWM_EN_Pin GPIO_PIN_15
#define PWM_EN_GPIO_Port GPIOB
#define U_PWM_Pin GPIO_PIN_8
#define U_PWM_GPIO_Port GPIOA
#define V_PWM_Pin GPIO_PIN_9
#define V_PWM_GPIO_Port GPIOA
#define W_PWM_Pin GPIO_PIN_10
#define W_PWM_GPIO_Port GPIOA
#define Power_Reset_Pin GPIO_PIN_12
#define Power_Reset_GPIO_Port GPIOA
#define Key_1_Pin GPIO_PIN_15
#define Key_1_GPIO_Port GPIOA
#define Key_2_Pin GPIO_PIN_3
#define Key_2_GPIO_Port GPIOB
#define Key_3_Pin GPIO_PIN_4
#define Key_3_GPIO_Port GPIOB
#define Key_4_Pin GPIO_PIN_5
#define Key_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

void uart_idleback(UART_HandleTypeDef *huart);	//串口空闲回调中断函数 具体定义放在usart.c

//电流采样变量
extern uint32_t ADC_Data[2];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
