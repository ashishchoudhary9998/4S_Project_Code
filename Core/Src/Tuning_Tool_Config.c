#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_usart.h"
#include "Tuning_Tool_Config.h"
#include "usart.h"
#include "UV_Recovery.h"
#include "BQ76952.h"


#define NUM_CELLS 4  
/*Write System Tuning Parameters in Nonvolatile Memory. it only erase and re- write by tuning tool with fixed algorythm*/
static void Program_HalfWord(uint32_t Address, uint16_t Data);
HAL_StatusTypeDef FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);

#define   battery_capacity         s32aSystem_Tuning_Variable_Parameters[16]

extern void delay_1us(uint32_t count);

uint8_t check_1 = 0;

#pragma arm section rodata = "USER_AREA_ROM"

const int64_t s32aSystem_Tuning_Fixed_Parameters[26] = { 
																											
	                                              /*0*/      3600,     /*cell volt high cut*/          //F7C        
																								/*1*/			 2500,			/*cell volt low cut*/	  //F80
																								/*2*/			 14400,		/* Pack volt high cut*/                 //F84
																							  /*3*/      12000, /*Pack volt low cut*/         //F88
																							  /*4*/      60,    /*chg overcurrent protection*/   //F8C
																								/*5*/			 110,     /*dischg overcurrent protection*/ //F90
																								/*6*/				0x4765,		/*Serial 1*/     //F94
																								/*7*/				0x6E75,			/*Serial 2*/               //F98
																								/*8*/				0x7320,			/*Serial 3*/          //F9C
																								/*9*/				0x424D,			/*Serial 4*/		        //FA0
																								/*10*/			0x5300, 	/*Serial 5*/ //FA4
	                                              /*11*/      3450,   /*balance start volt*/   // FA8
                                                /*12*/       65,  /*chg high temp protection*/  //FAC
                                                /*13*/       -10 , /*chg low temp protection*/  //FB0
                                                /*14*/       75, /*dischg high temp protection*/  // FB4
                                                /*15*/       -20,// /*dischg low temp protection*/ //FB8
                                                /*16*/       100, /*Battery Capacity, Capacity in Ah*/ 	// FBC
	                                              /*17*/       0000, /*Serial 6*/  //FC0
																								/*18*/       0000, /*Serial 7*/  //FC4
																								/*19*/       1234, /**/ //FC8
																								/*20*/       336, /*chg current tunning*/ 	// FCC //402 //201-10 //232-12//120
																								/*21*/       313, /*dischg current tunning*/ 	//1.2 -2 //122 //FD0
																								/*22*/       800, /*difference volt tunning*/ 	// FD4
																								/*23*/       125, /*Mosfet temp protect*/  	// FD8
																								/*24*/       20, /*Balance Diff volt*/  	// 20_00_10_00 //FDC
																								/*25*/       0000, /*Serial 8*/  //FE0
																											 };
#pragma arm section code
/*end*/																											 

int16_t u32aTool_Display_Parameters[69] __attribute__((at(0x20000E1C)));																											 																											 																											  																											   
int32_t s32aSystem_Tuning_Variable_Parameters[26] __attribute__((at(0x20000F7C)));//28	
																											 
/***************************************************************************************/
uint32_t FirstPage = 0, NbOfPages = 0,BankNumber = 0;
uint32_t Address = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
/***************************************************************************************/																											 

uint16_t calculateCRC(uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;

  for (uint16_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos]; // XOR byte into least significant byte of crc

    for (uint8_t i = 8; i != 0; i--) { // Loop over each bit
      if ((crc & 0x0001) != 0) { // If the LSB is set
        crc >>= 1; // Shift right
        crc ^= 0xA001; // XOR with the polynomial
      } else {
        crc >>= 1; // Just shift right
      }
    }
  }
  return crc;
}

/*************************************************************************************/
uint8_t tuning_response[143];
uint16_t registers[100] = {0};  // Assuming we have 100 registers
uint16_t tuning_crc;
uint16_t received_tuning_crc; //, data, error_flag = 0;
uint8_t tuning_byteCount;
int16_t startAddress;
int16_t registerCount;
int16_t crcReceived;
int16_t crcCalculated;
uint16_t readRegister(uint16_t* registerBase, uint16_t address);
void _delay(int delay_count);
uint16_t RS485_count_flag = 0;
uint32_t base_address = 0x20000000, address_32bit;
uint16_t address_16bit_data1,value;
uint8_t Tuning_RX_Flag=0;
extern uint8_t Tuning_query[8];
uint8_t test_array=0;

int16_t signed_value;
/*************************************************************************************/


void ProcessTuningQuery(uint8_t *query, uint16_t length) 
	{
	
  if (length != 8U) return;
  // Extract received CRC
  crcReceived = (Tuning_query[length - 1] << 8) | Tuning_query[length - 2];

  // Calculate CRC of received data (excluding received CRC)
  crcCalculated = calculateCRC(Tuning_query, length - 2);

  // Validate CRC
  if (crcReceived != crcCalculated) {
    return; // CRC mismatch, ignore request
  }

  // Clear previous response
  //for (int clear_count = 0; clear_count < 115; clear_count++) {
   // response[clear_count] = 0;
 // }

  // Extract start address and register count
  startAddress = (Tuning_query[2] << 8) | Tuning_query[3];
  registerCount = (Tuning_query[4] << 8) | Tuning_query[5];

  // Calculate byte count
  tuning_byteCount = registerCount * 2;

  if (Tuning_query[0] == 0x01) {
    if (Tuning_query[1] == 0x03) { // Function 03 (03hex) Read Holding Registers
      // Build response
      tuning_response[0] = Tuning_query[0];
      tuning_response[1] = Tuning_query[1];
      tuning_response[2] = tuning_byteCount;

      for (int i = 0; i < registerCount; i++) {
        address_32bit = base_address + startAddress + (i * 2);
        value = *(uint16_t*)address_32bit;
        tuning_response[3 + i * 2] = value >> 8;
        tuning_response[3 + i * 2 + 1] = value & 0xFF;
      }

      // Calculate CRC
      tuning_crc = calculateCRC(tuning_response, 3 + tuning_byteCount);
      tuning_response[3 + tuning_byteCount] = tuning_crc & 0xFF;
      tuning_response[4 + tuning_byteCount] = tuning_crc >> 8;

      // Send response
			Tuning_RX_Flag=1;
      HAL_UART_Transmit(&huart1, tuning_response, 5 + tuning_byteCount, 100);

    } 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		else if (Tuning_query[1] == 0x06) 
		{ // Function 06 (06hex) Write Single Register
      startAddress = (Tuning_query[2] << 8) | Tuning_query[3];
      uint32_t address = base_address + startAddress;
      uint16_t value = (Tuning_query[4] << 8) | Tuning_query[5];
			
		//	if (startAddress == 0x0FB0 || startAddress == 0x0FB8)
   // {
		//	test_array++;
        // CORRECT: Just cast to int16_t — NO +65536!
      //   signed_value = (int16_t)value;
      //  *(int16_t *)address = signed_value;
  //  }
			
      if (startAddress == UV_RESET_ADDRESS)
      {
          /* Command register: consume in RAM, do not persist or reset in ISR. */
          s32aSystem_Tuning_Variable_Parameters[UV_RESET_INDEX] = value;
          if (value == UV_RESET_VALUE)
              uv_reset_requested = 1;
          s32aSystem_Tuning_Variable_Parameters[UV_RESET_INDEX] = 0;
      }
      else
          *(uint16_t *)address = value;

      // Build response
      for (int i = 0; i < 6; i++) {
        tuning_response[i] = Tuning_query[i];
      }

      // Calculate CRC
      tuning_crc = calculateCRC(tuning_response, 6);
      tuning_response[6] = tuning_crc & 0xFF;
      tuning_response[7] = (tuning_crc >> 8) & 0xFF;
    }
  }
}
																											 																											
extern uint16_t u16Battery_Volt;
extern uint16_t u16DC_Charging_Current,u16Dischrging_Current;
extern uint16_t u16Battery_48V;
extern uint8_t u8solarflag;
extern uint8_t u8mainsflag;
extern uint16_t u16Total_Stack_Voltage;
extern uint16_t u16Cell_Numbers15,u16Dis_Off_Cntr,Low_Cell;
extern uint16_t u16Load_On_Current_Ctr, u16Load_Onn_Cnt;	
extern uint16_t Cell_Temp1,Cell_Temp2,Cell_Temp3,u16Load_status,u16Charg_status;//largest_number,lower_number,CellDiff;
extern int16_t  CellVoltDiff, Max_Cell_Voltage, Min_Cell_Voltage;
extern uint16_t CellVoltage[17];
extern double amb_Temp,mosfet_temp,temperature1, temperature2, temperature3, temperature4;
extern uint8_t Max_Cell_Voltage_Num, Min_Cell_Voltage_Num, Battery_State, Cell_balancing_state;
//extern uint8_t sync, final_soc, error_flag1;
extern uint8_t sync, final_soc;
extern float Internal_Resistance, Cell_Internal_Resistance[16];
extern uint8_t App_final_chg_current, App_final_dis_current;

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


void Update_Tool_Display(void)
{
	u32aTool_Display_Parameters[0] =  CellVoltage[0]; //E1C
	u32aTool_Display_Parameters[1] =  CellVoltage[1]; //E20
	u32aTool_Display_Parameters[2] =  CellVoltage[2]; //E24
	u32aTool_Display_Parameters[3] =  CellVoltage[3]; //E28
	u32aTool_Display_Parameters[4] =  CellVoltage[4]; //E2C
	u32aTool_Display_Parameters[5] =  CellVoltage[5]; //E30
	u32aTool_Display_Parameters[6] =  CellVoltage[6];  //E34
	u32aTool_Display_Parameters[7] =  CellVoltage[7];  //E38
	u32aTool_Display_Parameters[8] =  CellVoltage[8];
	u32aTool_Display_Parameters[9] =  CellVoltage[9]; 
	u32aTool_Display_Parameters[10] =  CellVoltage[10]; 
	u32aTool_Display_Parameters[11] =  CellVoltage[11]; 
	u32aTool_Display_Parameters[12] =  CellVoltage[12]; 
	u32aTool_Display_Parameters[13] =  CellVoltage[13]; 
	u32aTool_Display_Parameters[14] =  CellVoltage[14]; 
	u32aTool_Display_Parameters[15] =  CellVoltage[15];  
	
	u32aTool_Display_Parameters[16] = Max_Cell_Voltage; //E5C
	u32aTool_Display_Parameters[17] = Max_Cell_Voltage_Num+1; //E60
	u32aTool_Display_Parameters[18] = Min_Cell_Voltage;  //E64
	u32aTool_Display_Parameters[19] = Min_Cell_Voltage_Num+1; //E68
	u32aTool_Display_Parameters[20] = temperature[0];  //E6C
	u32aTool_Display_Parameters[21] = temperature[1]; 
	u32aTool_Display_Parameters[22] = temperature[2]; 
	u32aTool_Display_Parameters[23] = temperature[3]; 
	
	u32aTool_Display_Parameters[24] = 0;      //E7C
	u32aTool_Display_Parameters[25] = mosfet_temp;   //E80
	
  u32aTool_Display_Parameters[26] = u16Total_Stack_Voltage;	 //E84  
	u32aTool_Display_Parameters[27] = Charging_Current*10;// E88
  u32aTool_Display_Parameters[28] = u16Discharging_Current*10;// E8C
	u32aTool_Display_Parameters[29] = CHG; //E90
	u32aTool_Display_Parameters[30] = DSG;  //E94
	u32aTool_Display_Parameters[31] = RAW_Cell_Balancing_State; //E98
	u32aTool_Display_Parameters[32] = voltage_delta; //E9C
  u32aTool_Display_Parameters[33] = Power_W; //EA0
	u32aTool_Display_Parameters[34] = Total_cycles; //EA4
	u32aTool_Display_Parameters[35] =  soh; //E22
	u32aTool_Display_Parameters[36] = final_soc; //EA8
	u32aTool_Display_Parameters[37] = Remaining_Capacity; //EAC
	u32aTool_Display_Parameters[38] = error_flag1; //EB0
	u32aTool_Display_Parameters[39] = NUM_CELLS; //EB0
	u32aTool_Display_Parameters[40] = battery_capacity; //Battery Capacity //EB0
	u32aTool_Display_Parameters[41] = BackupTime_minutes; //20000E66 
	u32aTool_Display_Parameters[42] = s32aSystem_Tuning_Variable_Parameters[6]; //20000F28 // Serial No
	u32aTool_Display_Parameters[43] = s32aSystem_Tuning_Variable_Parameters[7]; //20000F2C // Serial No
	u32aTool_Display_Parameters[44] = s32aSystem_Tuning_Variable_Parameters[8]; //20000F30 // Serial No
	u32aTool_Display_Parameters[45] = s32aSystem_Tuning_Variable_Parameters[9]; //20000F34 // Serial No
	u32aTool_Display_Parameters[46] = s32aSystem_Tuning_Variable_Parameters[10]; //20000F38 // Serial No
	u32aTool_Display_Parameters[47] = s32aSystem_Tuning_Variable_Parameters[17]; //20000F3C // Serial No
	u32aTool_Display_Parameters[48] = s32aSystem_Tuning_Variable_Parameters[18]; //20000F40 // Serial No
	u32aTool_Display_Parameters[49] = s32aSystem_Tuning_Variable_Parameters[0]; //Cell High Cut
	u32aTool_Display_Parameters[50] = s32aSystem_Tuning_Variable_Parameters[1]; //Cell Low Cut
	u32aTool_Display_Parameters[51] = s32aSystem_Tuning_Variable_Parameters[11]; //Balance Start Volt
	u32aTool_Display_Parameters[52] = s32aSystem_Tuning_Variable_Parameters[22]; //Difference Volt Protection
	u32aTool_Display_Parameters[53] = s32aSystem_Tuning_Variable_Parameters[12]; //CHG High Temp
	u32aTool_Display_Parameters[54] = s32aSystem_Tuning_Variable_Parameters[13]; //CHG Low Temp
	u32aTool_Display_Parameters[55] = s32aSystem_Tuning_Variable_Parameters[14]; //DSG High Temp
	u32aTool_Display_Parameters[56] = s32aSystem_Tuning_Variable_Parameters[15]; //DSG Low Temp
	u32aTool_Display_Parameters[57] = s32aSystem_Tuning_Variable_Parameters[23]; //Mosfet Temp Protection
	u32aTool_Display_Parameters[58] = s32aSystem_Tuning_Variable_Parameters[24]; //Balance Diff Volt
	u32aTool_Display_Parameters[59] = s32aSystem_Tuning_Variable_Parameters[4]; //20000F0C  //Max CHG Current
	u32aTool_Display_Parameters[60] = s32aSystem_Tuning_Variable_Parameters[5]; //20000F10  //Max Dischg Current
	u32aTool_Display_Parameters[61] = s32aSystem_Tuning_Variable_Parameters[3]; //20000F14  //Battery Low Cut
	u32aTool_Display_Parameters[62] = s32aSystem_Tuning_Variable_Parameters[2]; //20000F18  //Battery High Cut
	//u32aTool_Display_Parameters[63] = Warnings;
	u32aTool_Display_Parameters[64] = s32aSystem_Tuning_Variable_Parameters[20]; //Charge Current Tuning
	u32aTool_Display_Parameters[65] = s32aSystem_Tuning_Variable_Parameters[21]; //Discharge Current Tuning
	u32aTool_Display_Parameters[66] = 10; //Firmware Revision
	u32aTool_Display_Parameters[67] = 0001; //Serial Number8
	u32aTool_Display_Parameters[68] = 11; //Hardware Revision
}

void Copy_Flash_To_Ram(int32_t *s32pram_address, int64_t *s32pflash_address, uint8_t u8size)
{
	uint8_t u8size_cntr = 0;
            
	
	while(u8size_cntr++ < u8size)	*s32pram_address++ =(int32_t) *s32pflash_address++; //chenge here typecasting
} 


//#define FLASH_USER_START_ADDR   (uint32_t)0x0801F000   /* Start @ of user Flash area */
//#define FLASH_USER_END_ADDR      0x0801F1F0   /* End @ of user Flash area */
//#define FLASH_USER_END_ADDR      (uint32_t)0x0801F7FF   /* End @ of user Flash area */

#define FLASH_USER_START_ADDR   (uint32_t)0x08007C00   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR      0x08007CF0  /* End @ of user Flash area */
 

uint32_t u32Address = 0, PageError = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

enum{

	FLASH_WRITE_UNKNOWN 	= 0,
	FLASH_WRITE_VERIFYING = 1,
	FLASH_WRITE_NOT_OK 		= 2,
	FLASH_WRITE_OK 				= 3,

}FLASH_WRITE_STATUS;

/*********************************************************************************************/
static uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}

static uint32_t GetBank(uint32_t Addr)
{
  return FLASH_BANK_1;
}

/*********************************************************************************************/
uint8_t Flash_Erase_and_Program(void)
{
	uint8_t u8data_index = 0,u8status = FLASH_WRITE_UNKNOWN;
//	uint32_t *u32psrc_address_ptr;
	int32_t *u32psrc_address_ptr;
	int64_t *u32pdest_address_ptr; //changes by int64_t
	uint8_t u32word_cntr = 0;
	
	HAL_FLASH_Unlock();
/*******************************************************************************************/
	/* Get the 1st page to erase */
  FirstPage = GetPage(FLASH_USER_START_ADDR);

  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
	
	/* Get the bank */
  BankNumber = GetBank(FLASH_USER_START_ADDR);

	/*****************************************************************************************/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
  EraseInitStruct.Page = FirstPage;//FLASH_USER_START_ADDR;
  EraseInitStruct.NbPages = 3;//(FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

	/*****************************************************************************************/

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    /*
      Error occurred while page erase.
      User can add here some code to deal with this error.
      PageError will contain the faulty page and then to know the code error on this page,
      user can call function 'HAL_FLASH_GetError()'
    */
    /* Infinite loop */
    while (1)
    {}
	
	}

	u32Address = FLASH_USER_START_ADDR;
	
	while (u32Address < FLASH_USER_END_ADDR)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, u32Address, s32aSystem_Tuning_Variable_Parameters[u8data_index++]) == HAL_OK)
    {
     // u32Address = u32Address + 4;
			u32Address = u32Address + 8;
			
    }
    else
    {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
      while (1){}
		
		}
  }

	u32psrc_address_ptr = (int32_t *)&s32aSystem_Tuning_Variable_Parameters[0];
	u32pdest_address_ptr = (int64_t *)&s32aSystem_Tuning_Fixed_Parameters[0];
	
	u8status  = FLASH_WRITE_VERIFYING;
	
	while(*u32psrc_address_ptr++ == *u32pdest_address_ptr++)
	//while(s32aSystem_Tuning_Variable_Parameters[u32word_cntr] == s32aSystem_Tuning_Fixed_Parameters[u32word_cntr])
	{
		if(++u32word_cntr >= 25)
		
		{
			u8status  = FLASH_WRITE_OK;
			break;
		}
		else u8status  = FLASH_WRITE_NOT_OK;
	}
	
	HAL_FLASH_Lock();

	return u8status;
}

/***************************************************************************************************/
/**
  * @brief  Program a half-word (16-bit) at a specified address.
  * @param  Address specify the address to be programmed.
  * @param  Data    specify the data to be programmed.
  * @retval None
  */
static void Program_HalfWord(uint32_t Address, uint16_t Data)
{
  /* Clean the error context */
  pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;
  
    /* Proceed to program the new data */
    SET_BIT(FLASH->CR, FLASH_CR_PG);

  /* Write data in the address */
  *(__IO uint16_t*)Address = Data;
}

/**
  * @brief  Program halfword, word or double word at a specified address
  * @note   The function HAL_FLASH_Unlock() should be called before to unlock the FLASH interface
  *         The function HAL_FLASH_Lock() should be called after to lock the FLASH interface
  *
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @note   FLASH should be previously erased before new programmation (only exception to this 
  *         is when 0x0000 is programmed)
  *
  * @param  TypeProgram   Indicate the way to program at a specified address.
  *                       This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address       Specifie the address to be programmed.
  * @param  Data          Specifie the data to be programmed
  * 
  * @retval HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t index = 0U;
  uint8_t nbiterations = 0U;
  
  /* Process Locked */
  __HAL_LOCK(&pFlash);

  /* Check the parameters */
  assert_param(IS_FLASH_TYPEPROGRAM(TypeProgram));
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
  
  if(status == HAL_OK)
  {
    if(TypeProgram == PROGRAM_HALFWORD)
    {
      /* Program halfword (16-bit) at a specified address. */
      nbiterations = 1U;
    }
    else if(TypeProgram == PROGRAM_WORD)
    {
      /* Program word (32-bit = 2*16-bit) at a specified address. */
      nbiterations = 2U;
    }
    else
    {
      /* Program double word (64-bit = 4*16-bit) at a specified address. */
      nbiterations = 4U;
    }

    for (index = 0U; index < nbiterations; index++)
    {
      Program_HalfWord((Address + (2U*index)), (uint16_t)(Data >> (16U*index)));

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
    
        /* If the program operation is completed, disable the PG Bit */
        CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
      /* In case of error, stop programation procedure */
      if (status != HAL_OK)
      {                         
        break;
      }
    }
  }

  /* Process Unlocked */
  __HAL_UNLOCK(&pFlash);

  return status;
}



/***************************************************************************************************/


/*EOF*/

