/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "UV_Recovery.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "BQ769x2Header.h"
#include "BQ76952.h"
#include "stm32g0xx_hal_spi.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_it.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CONVERTED_DATA_BUFF_SIZE 	8
uint16_t aADC_Converted_Data[ADC_CONVERTED_DATA_BUFF_SIZE];
uint32_t aADC_Converted_Data_Buff[ADC_CONVERTED_DATA_BUFF_SIZE];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t sync =35;
//Ambient temperature
float amb_volt, amb_volt_flt;
int int_amb_volt;
double amb_resistance1, amb_res_ln1, amb_Temperature;
double amb_Temp;
const int SAMPLE_NUMBER = 15;
int num[10], sum ;
float	avg;

//Mosfet Temperature
float mosfet_volt, mosfet_volt_flt;
int int_mosfet_volt;
double mosfet_resistance, mosfet_res_ln, mosfet_temperature;
double mosfet_temp;
const int SAMPLE_NUMBER1 = 15;
int num1[10], sum1 ; 
float	avg1; 

extern volatile uint8_t scheduler_flag;
extern volatile uint8_t read_bq76952_flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay_1us(uint32_t count);
void ADC_Sampling(void);
void Amb_Temp(void);
void mosfet_Temp(void);

extern void Copy_Flash_To_Ram(int32_t *s32pram_address, /*int32_t*/int64_t *s32pflash_address, uint8_t u8size);
extern const int64_t s32aSystem_Tuning_Fixed_Parameters[26];	
extern int32_t s32aSystem_Tuning_Variable_Parameters[26];
extern uint8_t Flash_Erase_and_Program(void);
extern void Task_Scheduler(void);
extern void CAN_Tranmitter(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void read_last_SOC_and_Cycles(uint8_t *soc, uint16_t *cycles, uint8_t *flag);

void AFE_Init(void)
{
  HAL_Delay(1000);
  CommandSubcommands(BQ769x2_RESET);  // Resets the BQ769x2 registers
	delay_1us(60000);
	BQ769x2_Init();  // Configure all of the BQ769x2 register settings
	delay_1us(10000);
	CommandSubcommands(FET_ENABLE); // Enable the CHG and DSG FETs
	delay_1us(10000);
	
	//CommandSubcommands(LOAD_DETECT_ON); // Enable Load Detection
	//delay_1us(10000);
	
	CommandSubcommands(SLEEP_DISABLE); // Sleep mode is enabled by default. For this example, Sleep is disabled to 
									   // demonstrate full-speed measurements in Normal mode.  //SLEEP_DISABLE  SLEEP_ENABLE

	delay_1us(60000); delay_1us(60000); delay_1us(60000); delay_1us(60000);  //wait to start measurements after FETs close
	
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
  UV_Recovery_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  //MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	
	
	LL_USART_EnableIT_RXNE(USART1);		/*ENABLE UART Receiver Full Interrupt Error*/
	LL_USART_Enable(USART1);
	
	LL_USART_EnableIT_RXNE(USART2);		/*ENABLE UART Receiver Full Interrupt Error*/
	LL_USART_Enable(USART2);
	
	AFE_Init();
	
  Copy_Flash_To_Ram((int32_t *)&s32aSystem_Tuning_Variable_Parameters[0],(int64_t *)&s32aSystem_Tuning_Fixed_Parameters[0],26);
	
	
	HAL_ADCEx_Calibration_Start(&hadc1);			//Calibrate to ADC before starting sampling of channel hadc1 instead of hadc
	LL_ADC_Enable(ADC1);
	while (!(LL_ADC_IsActiveFlag_ADRDY(ADC1)))
	{
	}
	
	LL_ADC_REG_SetSequencerChannels(ADC1,LL_ADC_CHANNEL_0|LL_ADC_CHANNEL_1|LL_ADC_CHANNEL_2			/*Set Channel Sequence for sampling*/
																	|LL_ADC_CHANNEL_3|LL_ADC_CHANNEL_4|LL_ADC_CHANNEL_5|LL_ADC_CHANNEL_7|LL_ADC_CHANNEL_8);
	
	LL_ADC_REG_SetDMATransfer(ADC1,LL_ADC_REG_DMA_TRANSFER_UNLIMITED);			/*Set DMA for ADC */
	
	/*Set DMA Transfer for Source to Destination*/
	LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,			/*Set DMA Channel*/
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA), /*peripheral Address*/
                         (uint32_t)&aADC_Converted_Data,		/*Memory Address*/
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);	/*Data Flow Direction*/
	
	LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1, ADC_CONVERTED_DATA_BUFF_SIZE);  /*Set DMA Transfer Size*/
	
  LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);	/*Ebale DMA*/

  LL_TIM_SetUpdateSource(TIM14,LL_TIM_UPDATESOURCE_COUNTER);  /*Update Source Selection*/
	LL_TIM_EnableIT_UPDATE(TIM14);
	LL_TIM_EnableCounter(TIM14);
	
	read_last_SOC_and_Cycles(&final_soc, &Total_cycles, &SOC_init_flag);
	soc = final_soc/100.0;
	soc_prev = soc;
	
	AFE_Init();
	MX_IWDG_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		UV_Recovery_Service(); HAL_IWDG_Refresh(&hiwdg);  // Refresh the watchdog timer 3 seconds
		
		if (scheduler_flag)
    {
     scheduler_flag = 0;
     Task_Scheduler();  // Execute tasks outside ISR
    }
		if (read_bq76952_flag) //Read voltage/current in background
    {
     read_bq76952_flag = 0;
     BQ76952_Read_Voltage_Current_temp();
    }
		BQ769x2_ServiceCommErrors();
		
		if(s32aSystem_Tuning_Variable_Parameters[19] == 4321)
		{
			s32aSystem_Tuning_Variable_Parameters[19] = 1234;	
			LL_TIM_DisableIT_UPDATE(TIM14);
			LL_USART_DisableIT_RXNE(USART1);
			
			s32aSystem_Tuning_Variable_Parameters[18] = (int32_t)Flash_Erase_and_Program();
			
			if(s32aSystem_Tuning_Variable_Parameters[18] == 3)
				
			Copy_Flash_To_Ram((int32_t *)&s32aSystem_Tuning_Variable_Parameters[0],(int64_t *)&s32aSystem_Tuning_Fixed_Parameters[0],26);  
			LL_TIM_EnableIT_UPDATE(TIM14);
			LL_USART_EnableIT_RXNE(USART1);
		}
	
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void delay_1us(uint32_t count)//1us//70//14ns per instruction
	{
	  while(count!=0)
		{
		 for(int i=0;i<40;i++) //40
		 __ASM volatile ("NOP");
		 count--;
	  }
	}
	
void ADC_Read_Temp()
{
		LL_ADC_REG_StartConversion(ADC1);					/*Start ADC Conversion*/
				
		while(!LL_DMA_IsActiveFlag_TC1(DMA1));		/*wait for ADC conversion and DMA Request completion*/
		
		LL_DMA_ClearFlag_GI1(DMA1);								/*Clear DMA Flag*/
	  ADC_Sampling();	
}

void ADC_Sampling(void)
{
	aADC_Converted_Data_Buff[0] +=  aADC_Converted_Data[0];
	
	aADC_Converted_Data_Buff[1] +=  aADC_Converted_Data[1];
	aADC_Converted_Data_Buff[2] +=  aADC_Converted_Data[2];
	aADC_Converted_Data_Buff[3] +=  aADC_Converted_Data[3];
  aADC_Converted_Data_Buff[4] +=  aADC_Converted_Data[4];
	aADC_Converted_Data_Buff[5] +=  aADC_Converted_Data[5];
}


void Amb_Temp(void)
{
	// amb_volt = aADC_Converted_Data[0]*(3.3/4096);
	 //int_amb_volt = (amb_volt * 100);
	 
	  for (int i = 0; i < SAMPLE_NUMBER; ++i)
		{
		 amb_volt = aADC_Converted_Data[0]*(3.3/4096);
	   int_amb_volt = (amb_volt * 100);
		 num[i] = int_amb_volt;
     sum += num[i];
    }
		 
		 avg = sum / SAMPLE_NUMBER;
		 sum = 0;
	
	 amb_volt_flt = (float) avg;
	 amb_volt_flt = amb_volt_flt/100.00;
	
	 amb_resistance1 = 34*amb_volt_flt/(16.5-8.4*amb_volt_flt);	
	 amb_resistance1= amb_resistance1 * 1000 ; // Resistance in ohms  
	 amb_res_ln1 = log(amb_resistance1);
	 amb_Temperature = ( 1 / ( 0.001129148 + ( 0.000234125 * amb_res_ln1 ) + ( 0.0000000876741 * amb_res_ln1 * amb_res_ln1 * amb_res_ln1 ) ) );  //Temperature in Kelvin 
	 amb_Temperature =  amb_Temperature-273.15; // Temperature in degree Celsius 
   amb_Temp = (int)	amb_Temperature;	 
} 
	
void mosfet_Temp(void)
{
	 // mosfet_volt = aADC_Converted_Data[1]*(3.3/4096);
	 // int_mosfet_volt = (mosfet_volt * 100);
	
	  for (int i = 0; i < SAMPLE_NUMBER1; ++i)
		{
		 mosfet_volt = aADC_Converted_Data[1]*(3.3/4096);
	   int_mosfet_volt = (mosfet_volt * 100); 
		 num1[i] = int_mosfet_volt;
     sum1 += num1[i];
    }
		 
		 avg1 = sum1 / SAMPLE_NUMBER1;
		 sum1 = 0;
	
		mosfet_volt_flt = (float) avg1;
		mosfet_volt_flt = mosfet_volt_flt/100.00;
	
	  mosfet_resistance = 34*mosfet_volt_flt/(16.5-8.4*mosfet_volt_flt);	
	  mosfet_resistance= mosfet_resistance * 1000 ; // Resistance in ohms  
	  mosfet_res_ln = log(mosfet_resistance);
	  mosfet_temperature = ( 1 / ( 0.001129148 + ( 0.000234125 * mosfet_res_ln ) + ( 0.0000000876741 * mosfet_res_ln * mosfet_res_ln * mosfet_res_ln ) ) );  //Temperature in Kelvin 
	  mosfet_temperature =  mosfet_temperature-273.15; // Temperature in degree Celsius 
    mosfet_temp = (int)	mosfet_temperature;	
    if (mosfet_temp>=65)
		{
		 mosfet_temp = mosfet_temp-4;
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
