#include <stdio.h>
#include "main.h"

#define NUM_CELLS 4

typedef enum
{
	NA =0,
	OCD = 1,
	SCD =2,
	POV =3,
	PUV =4,
	OTD =5,
	OTC =6,
	UTD =7,
	UTC =8,
	COV =9,
	CUV=10,
	DeltaV =11,
	OCC =12,
	MOT =13,
	EOT =14,
	COW =15,
	Thermistor_Open =16,
} Error_Flag;

extern Error_Flag error_flag1;

/* Protection vetoes are independent of the single displayed error. */
uint8_t Charge_Protection_Active(void);
uint8_t Discharge_Protection_Active(void);
void OperateChargingMosfet(void);
void OperateLoadMosfet(void);

extern uint8_t	RAW_Cell_Balancing_State;   // Cell Balancing state
extern uint16_t voltage_delta;
extern uint16_t u16Total_Stack_Voltage;
extern uint16_t final_chg_current, final_dis_current,CAN_Current_data;
extern uint8_t Max_Temp, Max_Temp_No, Min_Temp, Min_Temp_No;
extern uint8_t final_soc;
extern uint8_t SOC_init_flag,soh;
extern uint16_t Total_cycles;
extern float soc,soc_prev ;
extern uint16_t Remaining_Capacity, current_capacity;
//extern uint8_t error_flag1;
extern uint16_t voltage_delta;
extern float u16Discharging_Current;
extern float Charging_Current;
extern double temperature [4];
extern uint8_t DSG;   
extern uint8_t CHG;   
extern uint8_t PCHG;  
extern uint8_t PDSG;  
extern float Total_Discharge_Ah;
extern float BackupTime_Hours;
extern float Remaining_Energy_kWh ;
extern uint16_t BackupTime_minutes;
extern float Power_W;
extern uint16_t Battery_Voltage ;
extern uint16_t Pack_Voltage;
extern uint16_t LD_Voltage;


extern void BQ769x2_Init(void);
extern void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen);
extern void CommandSubcommands(uint16_t command);
extern void Subcommands(uint16_t command, uint16_t data, uint8_t type);
extern void DirectCommands(uint8_t command, uint16_t data, uint8_t type);
extern void BQ769x2_Init(void);
extern void BQ769x2_ReadControlStatus(void);
extern void BQ769x2_ReadFETStatus(void);
extern void BQ769x2_ReadAlarmRAWStatus(void);
extern void BQ769x2_ReadSafetyStatus(void);
extern void BQ769x2_ReadPFStatus(void);
extern uint16_t BQ769x2_ReadVoltage(uint8_t command);
extern void BQ769x2_ReadAllVoltages(void);
extern uint16_t BQ769x2_ReadCurrent(void);
extern float BQ769x2_ReadTemperature(uint8_t command);
extern void BQ769x2_ReadPassQ(void);
extern void BQ769x2_ReadBatteryStatus(void);
extern void HostControlledBalancing(void);
extern void BQ76952_Read_Voltage_Current_temp(void);
extern void BQ769x2_BOTHOFF(void);
extern void BQ769x2_RESET_BOTHOFF(void);
extern void DFETOFF(void);
extern void CFETOFF(void);
extern void DFETON(void);
extern void CFETON(void);
extern void SOC_Estimation(void); 
extern void SOH_Estimation(void);
extern void Pre_Discharge(void);
extern void BQ769x2_ServiceCommErrors(void);
