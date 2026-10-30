#include "main.h"
#include "gpio.h"
#include <stdint.h>
#include <stdbool.h>
#include "BQ76952.h"

extern double mosfet_temp;
extern volatile uint16_t Warning_Counter;
extern  int32_t s32aSystem_Tuning_Variable_Parameters[26];

// Warning and Protection Thresholds
#define Pack_OV_WARNING       s32aSystem_Tuning_Variable_Parameters[2]-800  // Overvoltage warning (mV)
#define Pack_UV_WARNING       s32aSystem_Tuning_Variable_Parameters[3]+800   // Undervoltage warning (mV)
#define Cell_OV_WARNING       s32aSystem_Tuning_Variable_Parameters[0]-200  // Overvoltage warning (mV)
#define Cell_UV_WARNING       s32aSystem_Tuning_Variable_Parameters[1]+200   // Undervoltage warning (mV)
#define OCC_WARNING           s32aSystem_Tuning_Variable_Parameters[4]-2   // Overcharge current warning (mA)
#define ODC_WARNING           s32aSystem_Tuning_Variable_Parameters[5]+2    // Overdischarge current warning (mA)
#define Cell_OT_WARNING       s32aSystem_Tuning_Variable_Parameters[14]-15      // Over-temperature warning (°C)
#define Cell_UT_WARNING       s32aSystem_Tuning_Variable_Parameters[15]+10       // Under-temperature warning (°C)
#define Mosfet_OT_WARNING     s32aSystem_Tuning_Variable_Parameters[23]-15       // Over-temperature warning (°C)
#define Mosfet_UT_WARNING  10      // Under-temperature warning (°C)
#define LOW_SOC_WARNING 15  // Low SOC warning (%)


bool warning_active = false;
uint8_t Warning_Flag=0;
uint8_t check_counter=0;

// Function to check warnings
void Check_Warnings(void)
{
    warning_active = false;
	  check_counter++;

    if (u16Total_Stack_Voltage >= Pack_OV_WARNING && u16Total_Stack_Voltage < s32aSystem_Tuning_Variable_Parameters[2])
    {
        warning_active = true; //Pack OverVolt Warning
			  Warning_Flag=1;
    }
    if (u16Total_Stack_Voltage <= Pack_UV_WARNING && u16Total_Stack_Voltage > s32aSystem_Tuning_Variable_Parameters[3])
    {
        warning_active = true; //Pack UnderVolt Warning
			  Warning_Flag=2;
    }
		
		if (u16Total_Stack_Voltage >= Pack_OV_WARNING && u16Total_Stack_Voltage < s32aSystem_Tuning_Variable_Parameters[2])
    {
        warning_active = true; //Cell OverVolt Warning
			   Warning_Flag=3;
    }
    if (u16Total_Stack_Voltage <= Pack_UV_WARNING && u16Total_Stack_Voltage > s32aSystem_Tuning_Variable_Parameters[3])
    {
        warning_active = true; //Cell UnderVolt Warning
			  Warning_Flag=4;
    }

    if (final_chg_current >= OCC_WARNING && final_chg_current < s32aSystem_Tuning_Variable_Parameters[4])
    {
        warning_active = true; //Over Charge Current
			  Warning_Flag=5;
    }

    if (final_dis_current >= ODC_WARNING && final_dis_current < s32aSystem_Tuning_Variable_Parameters[5])
    {
        warning_active = true; //Over Discharge Current
			  Warning_Flag=6;
    }

    if (Max_Temp >= Cell_OT_WARNING && Max_Temp < s32aSystem_Tuning_Variable_Parameters[14])
    {
        warning_active = true; //Cell Over Temperature
			  Warning_Flag=7;
    }

    if (Min_Temp <= Cell_UT_WARNING && Min_Temp > s32aSystem_Tuning_Variable_Parameters[15])
    {
        warning_active = true; //Cell Under Temperature
			  Warning_Flag=8;
    }
		
		if (mosfet_temp >= Mosfet_OT_WARNING && mosfet_temp < s32aSystem_Tuning_Variable_Parameters[23])
    {
        warning_active = true; //Mosfet OverTemperature
			  Warning_Flag=9;
    }

    if (final_soc <= LOW_SOC_WARNING && final_soc > 10)
    {
        warning_active = true;
			  Warning_Flag=10; // Low SOC Warning
    }

    if (warning_active)
    {
			if(++Warning_Counter>3000)
			{
		 	 HAL_GPIO_TogglePin(GPIOA, Warning_LED_Pin); //Toggle Warning LED
		   HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET); //Buzzer ON
			 Warning_Counter=0;
			}
    }
    else
    {
			if(error_flag1!=0)
			{
			 HAL_GPIO_WritePin(GPIOA, Warning_LED_Pin, GPIO_PIN_RESET); //LED ON
			 HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET); //Buzzer OFF
			}
			if(error_flag1==0)
			{
      HAL_GPIO_WritePin(GPIOA, Warning_LED_Pin, GPIO_PIN_SET); //LED OFF
		  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET); //Buzzer OFF
			}
			Warning_Flag=0;
    }
}

