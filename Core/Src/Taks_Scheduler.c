#include "stm32g0xx_ll_gpio.h"
#include "Task_Scheduler.h"
#include "BQ76952.h"


uint8_t u8Second_Cntr,u8Second_Cntr_1;
volatile uint8_t read_bq76952_flag = 0;
extern uint16_t Convert_Sample_To_Actual(uint16_t *ptr16avg_value, uint16_t *ptr16pre_avg_value, uint8_t u8filter_size, uint16_t u16conversion_coffi);

extern void Update_Tool_Display(void);
extern void Load_MOSFET(void);
extern void OperateLoadMosfet(void);
extern void	 Temprature_calc(void);
extern void Tempertaure_Protection(void);
extern void Current_Prot(void);
extern void CellVoltageSampling(void);
uint16_t cell_voltages[16]; 
extern uint16_t u16Total_Stack_Voltage;
extern void OverVolt_Protection(void);
extern void OperateChargingMosfet(void);
extern void OperateLoadMosfet(void);
extern void Discharging_curr(void);
extern void Discharging_Current(void);
extern void Dischrging_Current_Prot(void);
extern void Overdisch_Current(void);
extern void Temprature_calc1(void);
void Tempertaure_Protection1(void);
void  Charging_Current_Prot(void);
//void OperatPWMTRIP(void);
void Short_Ckt(void);
extern void UnderVolt_Protection (void);
uint16_t u16Load_status,u16Charg_status;
extern uint16_t CellVoltage[19];
extern void Cell_Temp_Protection(void);
extern void Charge_State(void);
extern void Dharge_State(void);
extern void Short_Ckt_Prot(void);
extern void Amb_Temp(void);
extern void mosfet_Temp(void);
extern void Cell_Temp_low_Protection(void);

extern void Voltage_Current_protection(void);
extern void short_ckt_recovery(void);
extern void Thermal_check(void);
extern void open_wire_detection(void);
extern void Thermistor_check(void);
extern void Open_wire_check(void);
extern void set_temp_protection(void);
extern void cell_volt_diff_protection(void);
extern void cel_volt_current_read(void);
extern void cel_temp_read(void);
extern void SOC_Display(void);
extern void Internal_Resistance_Calculation(void);
extern void ADC_Read_Temp(void);
extern void Load_Detection(void);
extern void sleep(void);
extern void Precharge_ON_OFF(void);
extern void Min_Max_Values(void);
extern void send_response(void);
extern void Check_Warnings(void);
extern void check_and_store_SOC_Cycles(uint8_t soc, uint16_t cycles, uint8_t flag);

extern volatile uint16_t LED_Counter;
extern void Update_Protection_Display(void);

extern enum
{
	eLoadOFF,
	eLoadON,
}LoadState;

extern enum
{
	eChargingOFF,
	eChargingON,
}CharginggState;

/*************************************************************************************************************************
Function:	Task_Scheduler 
Input:
Output:
Description:
Caller Fcution:
Called Funcation:
**************************************************************************************************************************/

void Task_Scheduler(void)
{
    static uint8_t ms1_state = 0, ms10_state = 0, ms100_state = 0, ms200_state = 0, ms500_state = 0, sec_state = 0;

    // 1ms Tasks
    switch (ms1_state)
    {
        case 0: 
					Update_Tool_Display(); 
				break;
        case 1: 
					OperateLoadMosfet();
					Pre_Discharge();
				break;
        case 2: 
					OperateChargingMosfet(); 
				break;
        case 3: 
					Dischrging_Current_Prot(); 
				break;
    }
    ms1_state = (ms1_state + 1) % 4; // Move to next state

    // 10ms Tasks
    static uint8_t ms1_counter = 0;
    if (++ms1_counter >= 10)
    {
        ms1_counter = 0;
        switch (ms10_state)
        {
            case 0: 
							OverVolt_Protection(); 
					    Cell_Temp_Protection();
					    Cell_Temp_low_Protection();
					  break;
            case 1: 
							UnderVolt_Protection(); 
					    //Tempertaure_Protection(); //Ambient Temp Protection
					    Tempertaure_Protection1(); //Mosfet Temp Protection
					  break;
					  case 2: 
						  Charging_Current_Prot();
						  cell_volt_diff_protection();
					    Update_Protection_Display();
						  //Open_wire_check();
						 break;
					  case 3: 
						BQ769x2_ReadFETStatus();
					   break;
        }
        ms10_state = (ms10_state + 1) % 4;
    }

    // 100ms Tasks
    static uint8_t ms10_counter = 0;
    if (++ms10_counter >= 10)
    {
        ms10_counter = 0;
        switch (ms100_state)
        {
				 case 0: 
					 read_bq76952_flag = 1; 
				 break;
         case 1: 
					 ////BQ769x2_ReadPassQ();	 
					// BQ769x2_ReadBatteryStatus();
				      BQ769x2_ReadAlarmRAWStatus();
					break;
				 case 2: 
	            SOC_Estimation(); 
				 break;
				 case 3: 
					// BQ769x2_ReadAlarmRAWStatus();
              ADC_Read_Temp(); 				 
				 break;
		     case 4: 
					 //BQ769x2_ReadControlStatus();
				   BQ769x2_ReadPFStatus();
				 break;
        }
        ms100_state = (ms100_state + 1) % 5;
    }
		
		// 200ms Tasks
    static uint8_t ms200_counter = 0;
    if (++ms200_counter >= 20) // 200ms (20 x 10ms)
    {
        ms200_counter = 0;
        switch (ms200_state)
        {
            case 0: // Add your 200ms task here
                HostControlledBalancing();
              //  Check_Warnings();						
                break;
            case 1: // Another 200ms task
             //   Thermistor_check();
						    check_and_store_SOC_Cycles(final_soc, Total_cycles, SOC_init_flag);	
                break;
						case 2: // Another 200ms task
                break;
						case 3: // Another 200ms task
                break;
        }
        ms200_state = (ms200_state + 1) % 2;
    }

    // 500ms Tasks
    static uint8_t ms500_counter = 0;
    if (++ms500_counter >= 50) // 500ms (50 x 10ms)
    {
        ms500_counter = 0;
        switch (ms500_state)
        {
            case 0: // Add your 500ms task here
               // Amb_Temp();
						    mosfet_Temp();
                break;
            case 1: // Another 500ms task
						    SOH_Estimation();
						    set_temp_protection();
                break;
        }
        ms500_state = (ms500_state + 1) % 2;
    }

    // 1 Second Tasks
    static uint8_t sec_counter = 0;
    if (++sec_counter >= 10)
    {
			if(++LED_Counter>3000)
			{
		//	SOC_Display();
			//LED_Counter =0;
			}
      sec_counter = 0;
    }
}
