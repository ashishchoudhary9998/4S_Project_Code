/* 
Take protection over Cell voltage low/high cut/revival for charge and load mosfet
  -Protection on Total stack voltage or cell voltage >= high cut (58.4, 3.7)
  -Protection on Total stack voltage and cell voltage <= high cut revival (54, 3.3)
  -Protection on Total stack voltage or cell voltage <= Low cut (40, 2.5)
	-Protection on Total stack voltage and cell voltage >= Low cut Revival (48, 3.0)
*/

#include "stm32g0xx_ll_adc.h"
#include "math.h"
#include "UV_Recovery.h"
#include "main.h"
#include "stm32g0xx_ll_gpio.h"
#include "BQ769x2Header.h"
#include "BQ76952.h"

#define B_gate_ON LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_7);//////////////////////Charging mosfet ON
#define B_gate_OFF LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_7);//////////////////////Charging mosfet OFF
#define BL_gate_ON LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_6);//////////////////////Discharging mosfet ON
#define BL_gate_OFF LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_6);//////////////////////Discharging mosfet OFF B_gate_ON BL_gate_ON


uint8_t Cell_Volt_High_Cut=0;
uint8_t Cell_Volt_High_Cut_Revive=0;
uint8_t Cell_Volt_Low_Cut=0;
uint8_t Cell_Volt_Low_Cut_Revive=0;

uint8_t Pack_Volt_High_Cut=0;
uint8_t Pack_Volt_High_Cut_Revive=0;
uint8_t Pack_Volt_Low_Cut=0;
uint8_t Pack_Volt_Low_Cut_Revive=0;

enum CHARGING_STATES{
	
	CHECK_CHARGING = 0,
	RUN_CHARGING,
	CHARGING_ERROR,
	CHARGING_ERROR1,
};
enum
{
	eChargingOFF,
	eChargingON,
}CharginggState; //ChargingState=eChargingOFF
enum
{
	eLoadOFF,
	eLoadON,
}LoadState;
enum DISCHRGING_STATES{
	
	CHECK_DISCHARGING = 0,
	RUN_DISCHARGING,
	DISCHARGING_ERROR,
};
enum
{
	eDischargingOFF,
	eDischargingON,
}DischargingState; //ChargingState=eChargingOFF
enum DISCHARGING_STATES{
	
	CHECK_DISCHR = 0,
	RUN_DISCHR,
	DISCHR_ERROR,
	
};

//extern uint32_t stack_voltage;
extern uint16_t u16Total_Stack_Voltage,u16DC_Charging_Current,u16Dischrging_Current;
uint8_t u16load_off_cnt,u16Load_On_Cntr7,u16load_off_cn;
uint8_t u16Delay_Cntr,u16Load_On_Cntr,u16Load_Off_Cntr,u16Load_On_Cntr,u16Load_Cntr,u16Pack_Load_Off_Cntr;
uint8_t Charging_State,Discharging_State;
extern uint8_t u8shortcktflag;
uint16_t Discharging_flag,Dischr_State;
uint8_t low_cel_vol_cn,low_pack_vol_cn;
uint16_t Retrive_cnt,u16Load_On_Cntr1;
//uint8_t error_flag=0;
//uint8_t error_flag1=0;
extern float u16Discharging_Current;
extern float Charging_Current;
extern  int32_t s32aSystem_Tuning_Variable_Parameters[26];

/*********************************************************************************************/
#define   Cell_Voltage_High_Cut         s32aSystem_Tuning_Variable_Parameters[0]
#define   Cell_Voltage_Low_Cut          s32aSystem_Tuning_Variable_Parameters[1]
#define   Toatal_Battery_Vol_High         s32aSystem_Tuning_Variable_Parameters[2]
#define   Toatal_Battery_Vol_Low         s32aSystem_Tuning_Variable_Parameters[3]

#define Cell_Voltage_Low_Cut_Revival    Cell_Voltage_Low_Cut +150 //2.65
#define   Cell_Voltage_High_Cut_Revival  Cell_Voltage_High_Cut-100 //3.55
#define   Toatal_Battery_Vol_Low_Retrive  Toatal_Battery_Vol_Low+600 //10.6
#define   Toatal_Battery_Vol_High_Retrive  Toatal_Battery_Vol_High-400 //14.2
/*********************************************************************************************/
//uint16_t cell0,cel0;
extern uint16_t CellVoltage[16];
extern uint8_t precharge_flag;
extern float soc,soc_prev ;

uint8_t highcut_flag =0, lowcut_flag =0, data_logging_start_flag=0;

//uint8_t test_flag;

extern void delay_1us(uint32_t count);
   
void OperateChargingMosfet(void)
{
	/* An ON request must never override another protection's active latch. */
	if ((CharginggState == eChargingON) && !Charge_Protection_Active())
	{
//	CommandSubcommands(ALL_FETS_ON);
	 //CFETON();
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);  // CFETOFF pin set High
	}
	else
	{
	//CommandSubcommands(CHG_PCHG_OFF);
	 //CFETOFF();
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);  // CFETOFF pin set Low
	}
}

void OperateLoadMosfet(void)
{
	if ((LoadState == eLoadON) && !Discharge_Protection_Active())
	{
	//CommandSubcommands(ALL_FETS_ON);
   //DFETON();
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);  // DFETOFF pin set High
	
	}
	else
	{
	//CommandSubcommands(DSG_PDSG_OFF);
	 //DFETOFF();
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);  // DFETOFF pin set Low
	}
}

void OverVolt_Protection(void)
{
	switch (Charging_State)
	{
	case CHECK_CHARGING:

	    if((u16Total_Stack_Voltage>4000)&&(u16Total_Stack_Voltage < 65000))
			{
			if(++u16Load_On_Cntr > 4)
			{
				
				CharginggState = eChargingON;
 				LoadState = eLoadON ;      //10-11
				Charging_State = RUN_CHARGING;
				u16Load_On_Cntr = 0;
			}
		}
		else
		{
			//	LoadState = eLoadOFF ;
			 // u16Load_On_Cntr = 0;
		}
		
	break;
		
	case RUN_CHARGING:
			
  for(int i=0; i<NUM_CELLS; i++)
	{
		if((CellVoltage[i]>Cell_Voltage_High_Cut) && (u16Discharging_Current ==0))
		{
		if(++u16Load_Off_Cntr>10) //100
		 {
		 Cell_Volt_High_Cut=1;
		 error_flag1 =COV; //High protection error
		 u16Load_Off_Cntr=0;
		 }
		}	
	}
	if((u16Total_Stack_Voltage>Toatal_Battery_Vol_High) && (u16Discharging_Current ==0))
		{
		 if(++u16Pack_Load_Off_Cntr>10) //100
		 {
		 Pack_Volt_High_Cut=1;
		 error_flag1=POV;
		 u16Pack_Load_Off_Cntr=0;
		 }
	  } 

		if(((Pack_Volt_High_Cut==1)||(Cell_Volt_High_Cut==1)))  //3700==3.7		
		{
			CharginggState = eChargingOFF;
			LoadState = eLoadON ;
			Charging_State = CHARGING_ERROR;
			
			if(highcut_flag ==0)
			{
		  soc = 1;
			soc_prev =1;
			highcut_flag =1;
			}
		}
			else 
		 {
			//LoadState = eLoadOFF ;
			//Charging_State = CHARGING_ERROR;				
		 } 
	
		break;
		
	case CHARGING_ERROR:
		
      if(((u16Total_Stack_Voltage <=Toatal_Battery_Vol_High_Retrive)&&((CellVoltage[0]<=Cell_Voltage_High_Cut_Revival)&&(CellVoltage[1]<=Cell_Voltage_High_Cut_Revival)
				&&(CellVoltage[2]<=Cell_Voltage_High_Cut_Revival)&&(CellVoltage[3]<=Cell_Voltage_High_Cut_Revival)))||(u16Discharging_Current>=1))
		{
				if(++u16Delay_Cntr>10) //20
				{
				error_flag1 =NA;
				CharginggState = eChargingON;	
				LoadState = eLoadON;  //10-11
				u16Delay_Cntr=0;
				Charging_State = RUN_CHARGING;
        Cell_Volt_High_Cut =0;	
        Pack_Volt_High_Cut =0;
				highcut_flag =0;
       // data_logging_start_flag=0;					
				}
			}	
			else
			{
			}
		break;
			
	default:
	  delay_1us(9000);
		CharginggState=eChargingOFF;
		LoadState = eLoadOFF;
		Charging_State = RUN_CHARGING;
		
		break;
	}
	}	
	/**************************************************************************************/
	
	
/* One-shot marker in unused backup register 0; never stored in tuning flash. */
#define UV_RESET_MARKER 0x55565231UL
volatile uint8_t uv_reset_requested = 0;
static uint8_t uv_recovery_active = 0;
static uint32_t uv_recovery_started = 0;

void UV_Recovery_Init(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    uv_recovery_active = ((TAMP->BKP0R == UV_RESET_MARKER) &&
                          ((RCC->CSR & RCC_CSR_SFTRSTF) != 0U));
    TAMP->BKP0R = 0U;
    HAL_PWR_DisableBkUpAccess();
    __HAL_RCC_CLEAR_RESET_FLAGS();
    uv_recovery_started = HAL_GetTick();
}

uint8_t recovery_counter = 0;
uint8_t protection_counter = 0;

/*
void UV_Recovery_Service(void)
{
    extern int32_t s32aSystem_Tuning_Variable_Parameters[26];
    if (uv_reset_requested)
    {
        uv_reset_requested = 0;
        s32aSystem_Tuning_Variable_Parameters[UV_RESET_INDEX] = 0;
        HAL_PWR_EnableBkUpAccess();
        TAMP->BKP0R = UV_RESET_MARKER;
        __DSB();
        NVIC_SystemReset();
        return;
    }
    if (!uv_recovery_active)
        return;

    if ((u16Discharging_Current > 3.0f) ||
        ((uint32_t)(HAL_GetTick() - uv_recovery_started) >= UV_RECOVERY_MS))
    {
        uint8_t discharge_abort = (u16Discharging_Current > 3.0f);
        uv_recovery_active = 0;
			  recovery_counter++;
        // Evaluate immediately at expiry, without the normal debounce delay. 
        for (int i = 0; i < NUM_CELLS; ++i)
            if (CellVoltage[i] < Cell_Voltage_Low_Cut)
                Cell_Volt_Low_Cut = 1;
        if (discharge_abort || (u16Total_Stack_Voltage < Toatal_Battery_Vol_Low))
            Pack_Volt_Low_Cut = 1;
        if ((Cell_Volt_Low_Cut || Pack_Volt_Low_Cut) && Charging_Current ==0)
        {
					  protection_counter++;
            error_flag1 = Cell_Volt_Low_Cut ? CUV : PUV;
            LoadState = eLoadOFF;
            Discharging_State = DISCHR_ERROR;
            Retrive_cnt = 0;
            OperateLoadMosfet();
            CommandSubcommands(DSG_PDSG_OFF);
        }
    }
}
*/

void UV_Recovery_Service(void)
{
    extern int32_t s32aSystem_Tuning_Variable_Parameters[26];
    if (uv_reset_requested)
    {
        uv_reset_requested = 0;
        s32aSystem_Tuning_Variable_Parameters[UV_RESET_INDEX] = 0;
        HAL_PWR_EnableBkUpAccess();
        TAMP->BKP0R = UV_RESET_MARKER;
        __DSB();
        NVIC_SystemReset();
        return;
    }
    if (!uv_recovery_active)
        return;

    /* 1. If charging current is present, recovery succeeded! 
          Do NOT set cut flags and do NOT take protection. */
    if (Charging_Current > 0) // Or use a noise threshold like (Charging_Current > 0.1f)
    {
        uv_recovery_active = 0; // Exit recovery mode normally
        return;
    }

    /* 2. Otherwise, check if discharge > 3A or timeout expired */
    if ((u16Discharging_Current > 3) ||
        ((uint32_t)(HAL_GetTick() - uv_recovery_started) >= UV_RECOVERY_MS))
    {
        uint8_t discharge_abort = (u16Discharging_Current > 3.0f);
        uv_recovery_active = 0;
        recovery_counter++;
        /* Only evaluate and set cut flags if NOT charging */
        for (int i = 0; i < NUM_CELLS; ++i)
        {
            if (CellVoltage[i] < Cell_Voltage_Low_Cut)
                Cell_Volt_Low_Cut = 1;
        }

        if (discharge_abort || (u16Total_Stack_Voltage < Toatal_Battery_Vol_Low))
            Pack_Volt_Low_Cut = 1;

        if (Cell_Volt_Low_Cut || Pack_Volt_Low_Cut)
        {
					  protection_counter++;
            error_flag1 = Cell_Volt_Low_Cut ? CUV : PUV;
            LoadState = eLoadOFF;
            Discharging_State = DISCHR_ERROR;
            Retrive_cnt = 0;
            OperateLoadMosfet();
            CommandSubcommands(DSG_PDSG_OFF);
        }
    }
}



void UnderVolt_Protection(void)
{
    UV_Recovery_Service();
    if (uv_recovery_active)
    {
        low_cel_vol_cn = 0;
        low_pack_vol_cn = 0;
        return;
    }
	switch (Discharging_State)
	{
	case CHECK_DISCHR:
	    
	   	if((u16Total_Stack_Voltage>=4000)&&(u16Total_Stack_Voltage<65000))
	   //   if(CellVoltage[0]>=Cell_Voltage_Low_Cut)
			{
			
			if(++u16Load_On_Cntr1 > 5)
			{
				CharginggState = eChargingON;
			//	LoadState = eLoadON ;       //10-11 //uncommented
				Discharging_State = RUN_DISCHR;
				u16Load_On_Cntr1 = 0;	
			}
		}
		else
		{
			
			LoadState = eLoadOFF ;
				u16Load_On_Cntr1 = 0;
		}
	break;
		
		
	case RUN_DISCHR:
		
	for(int i=0; i<NUM_CELLS; i++)
	{
		if((CellVoltage[i]<Cell_Voltage_Low_Cut) && (Charging_Current==0))
		{
		 if(++low_cel_vol_cn>8) //100
		 {
		 Cell_Volt_Low_Cut=1;
		 error_flag1 =CUV;
		 low_cel_vol_cn=0;
		 }
		}
	}	
  if((u16Total_Stack_Voltage<Toatal_Battery_Vol_Low) && (Charging_Current==0))
		{
		 if(++low_pack_vol_cn>8) //100
		 {
		 Pack_Volt_Low_Cut=1;
		 error_flag1 =PUV;
		 low_pack_vol_cn=0;
		 }
		}
	
	//	if((Pack_Volt_Low_Cut==1) ||Cell_Volt_Low_Cut==1)
		if(((Pack_Volt_Low_Cut==1) ||(Cell_Volt_Low_Cut==1)) && (Charging_Current==0))
		{
			CharginggState = eChargingON;
			LoadState = eLoadOFF ;
			CommandSubcommands(DSG_PDSG_OFF);
			Discharging_State = DISCHR_ERROR;
			
      if(lowcut_flag==0)
      {				
			 soc = 0;
			 soc_prev =0;
			 lowcut_flag=1;
			}
		}
			else 
		{
//				LoadState = eLoadON ;
			//  low_cel_vol_cn=0;	
       // Discharging_State = DISCHR_ERROR;			
			}
	
		break;
		
	case DISCHR_ERROR:
		 if(((u16Total_Stack_Voltage >=Toatal_Battery_Vol_Low_Retrive )&&( CellVoltage[0]>=Cell_Voltage_Low_Cut_Revival )&&( CellVoltage[1]>=Cell_Voltage_Low_Cut_Revival )&&( CellVoltage[2]>=Cell_Voltage_Low_Cut_Revival )&&( CellVoltage[3]>=Cell_Voltage_Low_Cut_Revival ))||(Charging_Current>1))
			{
				if(++Retrive_cnt>5) //100
				{
				error_flag1 =NA;
				CharginggState = eChargingON;	
				LoadState = eLoadON;
				CommandSubcommands(ALL_FETS_ON);	
				u16Delay_Cntr=0;
				Discharging_State = CHECK_DISCHR;
        Retrive_cnt=0;	
				Cell_Volt_Low_Cut=0;
				Pack_Volt_Low_Cut=0;
				data_logging_start_flag=0;
				lowcut_flag=0;
			 }
			}
			else
			{
		   	LoadState = eLoadOFF ;
			//	Discharging_State = CHECK_DISCHR;
			}
		
		break;
			
	default:
		CharginggState=eChargingOFF;
		LoadState = eLoadOFF;
		Discharging_State = CHECK_DISCHR;
		
		break;
	}	
}	
