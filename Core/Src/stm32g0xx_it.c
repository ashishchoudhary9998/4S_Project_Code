/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Task_Scheduler.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"
#include "BQ769x2Header.h"
#include "BQ76952.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Tuning_QUERY_LENGTH 8   // Set your expected query length
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint32_t u32temp_tx;
volatile uint16_t Tim14Cntr;
uint8_t short_ckt_counter =0,short_counter_flag =0;
volatile uint8_t scheduler_flag = 0;
volatile uint16_t LED_Counter=0;
volatile uint16_t Warning_Counter=0;

/*************************************************************************/
uint8_t Tuning_query[8];
uint8_t data_count=0;
uint8_t* gp_uart1_rx_address = Tuning_query;  // Pointer to the buffer
uint8_t g_uart1_rx_count = 0U;         // Counter for received bytes
uint8_t g_uart1_rx_length = Tuning_QUERY_LENGTH;  // Length of the expected data
uint8_t query_complete_flag =0;
/*************************************************************************/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//extern uint8_t UART_Tuning_Tool_Callback(uint8_t u8rx_char);
//extern uint8_t RS485_Data_Process(uint8_t Received_data);
extern void ProcessTuningQuery(uint8_t *query, uint16_t length); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern enum
{
	eChargingOFF,
	eChargingON,
}CharginggState; //ChargingState=eChargingOFF

extern enum
{
	eLoadOFF,
	eLoadON,
}LoadState;

//extern uint8_t error_flag1, precharge_flag, shrt_flag;
extern uint8_t precharge_flag, shrt_flag;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Short_ckt_Pin);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM14))  /*Check Source of Interrupt*/
	{
   	 LL_TIM_ClearFlag_UPDATE(TIM14);						/*clear interrupt status flag*/
		Tim14Cntr++;
		scheduler_flag = 1;  // Set a flag instead of calling Task_Scheduler()
		LED_Counter++;
		Warning_Counter++;
    //Task_Scheduler();
	

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */
		}
	else
	{	
		/*Clear Rest of Interrupt cause flags*/
		LL_TIM_ClearFlag_UPDATE(TIM14);
		LL_TIM_ClearFlag_BRK(TIM14);
		LL_TIM_ClearFlag_CC1(TIM14);
		LL_TIM_ClearFlag_CC1OVR(TIM14);
	}

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if (LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
  {
	  uint8_t rx_data = LL_USART_ReceiveData8(USART1);  // Read received data (RXNE flag is cleared by this)
       
		if(query_complete_flag==0)
				{
					// Check if the first byte is the expected start byte (0x01 in this case)
					if (rx_data == 0x01)
					{
						query_complete_flag=1;
						g_uart1_rx_count = 0U;                // Reset byte counter
						gp_uart1_rx_address = Tuning_query;          // Point to the start of the query buffer
					}
				} 
        // Store received byte into buffer and increment the pointer and counter
        if (g_uart1_rx_count < g_uart1_rx_length)
        {
            *gp_uart1_rx_address = rx_data;       // Store received byte
            gp_uart1_rx_address++;               // Move to the next position in the buffer
            g_uart1_rx_count++;                  // Increment the byte counter
            
            // If the entire query is received, trigger the receive complete callback
            if (g_uart1_rx_count == g_uart1_rx_length)
            {
							query_complete_flag=0;
							ProcessTuningQuery(Tuning_query, 8);
            }
        }
        else
        {
            // Software overrun handling (if more bytes received than expected)
            //r_Config_UART1_callback_softwareoverrun(rx_data);  // Call overrun callback
        }
				
				 // Clear USART errors
    if (LL_USART_IsActiveFlag_ORE(USART1)) { LL_USART_ClearFlag_ORE(USART1); }
    if (LL_USART_IsActiveFlag_FE(USART1))  { LL_USART_ClearFlag_FE(USART1);  }
    if (LL_USART_IsActiveFlag_NE(USART1))  { LL_USART_ClearFlag_NE(USART1);  }
    if (LL_USART_IsActiveFlag_PE(USART1))  { LL_USART_ClearFlag_PE(USART1);  }
			}

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
