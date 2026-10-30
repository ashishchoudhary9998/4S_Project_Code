/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define DFETOFF_Pin GPIO_PIN_14
#define DFETOFF_GPIO_Port GPIOC
#define CFETOFF_Pin GPIO_PIN_15
#define CFETOFF_GPIO_Port GPIOC
#define Amb_Temp_Pin GPIO_PIN_0
#define Amb_Temp_GPIO_Port GPIOA
#define MOS_Temp_Pin GPIO_PIN_1
#define MOS_Temp_GPIO_Port GPIOA
#define RS_485_TX_Pin GPIO_PIN_2
#define RS_485_TX_GPIO_Port GPIOA
#define RS_485_RX_Pin GPIO_PIN_3
#define RS_485_RX_GPIO_Port GPIOA
#define RS_485_DE_Pin GPIO_PIN_4
#define RS_485_DE_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_10
#define Buzzer_GPIO_Port GPIOA
#define Warning_LED_Pin GPIO_PIN_15
#define Warning_LED_GPIO_Port GPIOA
#define Short_ckt_Pin GPIO_PIN_5
#define Short_ckt_GPIO_Port GPIOB
#define Short_ckt_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
