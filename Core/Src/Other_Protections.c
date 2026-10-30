#include "main.h"
#include "stdbool.h"
#include "stm32g0xx_ll_gpio.h"
#include "BQ769x2Header.h"
#include "BQ76952.h"

enum CHAR_DISCH_STATES{
	
	CHECK_CHAR_DISCH = 0,
	RUN_CHAR_DISCH,
	CHAR_DISCH_ERROR,
	CHAR_DISCH_ERROR1,
};
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
enum DISCHRGING_STATES{
	
	CHECK_DISCHARGING = 0,
	RUN_DISCHARGING,
	DISCHARGING_ERROR,
};
extern enum
{
	eDischargingOFF,
	eDischargingON,
}DischargingState; //ChargingState=eChargingOFF


extern uint16_t u16Total_Stack_Voltage,cell_voltages[16];
extern uint16_t u16Delay_Cntr,u16Load_On_Cntr,u16Load_On_Cntr1,u16DC_Charging_Current,u16Dischrging_Current,u16Off_Ctr,u16Load_On_Current_Ctr;
extern uint8_t Charging_State,Discharging_State;
extern void OperateChargingMosfet(void);
extern void OperateLoadMosfet(void);
uint8_t Charging_Dischrging_State;
uint16_t Battery_Full_Charge_AH,Battery_Initial_Check_Cntr,State_Change_Cntr,Batt_Discharge_Rate,Battery_Health_Percentage;
int16_t Battery_Charge_AH,Milisecond_AH,Second_AH,Hour_AH;
bool Second_Flag, Minute_Flag;
int8_t Battery_Level_State;
extern uint8_t u8shortcktflag;
extern uint8_t thermal_condition1, thermal_condition2, thermal_condition3;
extern float u16Discharging_Current;
extern void delay_1us(uint32_t count);
extern void Load_Mosfet_Volt(void);
uint8_t precharge_flag = 0, shrt_flag =0, thermal_flag =0;
extern uint8_t temp_error_flag;
extern uint8_t open_wire_error_flag, cell_volt_diff_flag; //error_flag1;
extern uint8_t data_logging_start_flag;

uint16_t shrt_cntr, thermal_runaway_counter,open_wire_counter, volt_diff_counter, thermistor_counter,pre_charge_counter=0;

extern uint8_t short_ckt_counter,short_counter_flag;
uint8_t counter_counter =0;

extern  int32_t s32aSystem_Tuning_Variable_Parameters[26];
#define   difference_volt_protection         s32aSystem_Tuning_Variable_Parameters[22]

void short_ckt_recovery()
{
    if(short_counter_flag ==1)
		{
			if(++counter_counter>6)
			{
			short_ckt_counter =0;
			counter_counter =0;
			short_counter_flag =0;
			}
		}

    if (shrt_flag==1)	
	{
		HAL_GPIO_WritePin(GPIOA, Warning_LED_Pin, GPIO_PIN_SET); //LED ON
		HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET); //Buzzer ON
	}
	
	
	if (u16Discharging_Current <150 &&shrt_flag==1)
		{
			//if(++shrt_cntr>50000) //Counter for 3 min delay 
			if(++shrt_cntr>200)
			{
			error_flag1 =NA;
			precharge_flag = 0;
			shrt_flag =0;
			LoadState = eLoadON;
			CharginggState = eChargingON;
			HAL_GPIO_WritePin(GPIOA, Warning_LED_Pin, GPIO_PIN_RESET); //LED OFF
		  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET); //Buzzer OFF
			shrt_cntr =0;
			}
		}
}

uint8_t thermal_ON_counter =0;

void Thermal_check()
{
	if((thermal_condition1 ==1 && thermal_condition2==1) || (thermal_condition3 ==1 && thermal_condition2==1))
	{
		if(thermal_ON_counter>3)
		{
		//error_flag1 = 17; //Thermal runaway error
		LoadState = eLoadOFF;
		CharginggState=eChargingOFF;
		thermal_flag =1;
		thermal_ON_counter =0;
		}
	}
	
	if((thermal_condition1 ==0 || thermal_condition2==0) && (thermal_condition3 ==0 || thermal_condition2==0) && thermal_flag ==1)
	{
		if(++thermal_runaway_counter>10)
		{
		error_flag1 =NA;
		LoadState = eLoadON;
		CharginggState=eChargingON;
		thermal_flag =0;
		thermal_runaway_counter =0;
		}
	}
}

uint8_t open_wire_flag, cell_diff_flag, thermistor_open_flag, deltaV_Flag=0;

void Thermistor_check(void)
{
	
	if(temp_error_flag ==1)
	{
		error_flag1 = Thermistor_Open; //thermistor open error
		LoadState = eLoadOFF;
		CharginggState=eChargingOFF;
		thermistor_open_flag =1;
	}
	
	if(temp_error_flag ==0 && thermistor_open_flag ==1)
	{
		if(++thermistor_counter>10)
		{
		error_flag1 =NA;
		LoadState = eLoadON;
		CharginggState=eChargingON;
		thermistor_open_flag =0;
		thermistor_counter =0;
		}
	}
	
}

void Open_wire_check(void)
{
	
	if(voltage_delta>2000)
	{
		error_flag1 = COW; //open cell wire error
		LoadState = eLoadOFF;
		CharginggState=eChargingOFF;
	}
	
	else
	{
		if(++open_wire_counter>10 && error_flag1==0)
		{
		error_flag1 =NA;
		LoadState = eLoadON;
		CharginggState=eChargingON;
		open_wire_flag =0;
		open_wire_counter =0;
		}
	}
	
}

void cell_volt_diff_protection(void)
{
	if(voltage_delta >=difference_volt_protection)
	{
		error_flag1 = DeltaV; //cell difference errro
		LoadState = eLoadOFF;
		CharginggState=eChargingOFF;
		deltaV_Flag=1;
	}
	
	else
	{
		if(++volt_diff_counter>10 && deltaV_Flag==1)
		{
		error_flag1 =NA;
		LoadState = eLoadON;
		CharginggState=eChargingON;
		cell_diff_flag =0;
		volt_diff_counter =0;
		deltaV_Flag=0;
		}
	}
	
}
	/*********************************************************************************************************/	

