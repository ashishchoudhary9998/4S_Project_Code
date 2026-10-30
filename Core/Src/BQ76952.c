#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include <stdio.h>
#include "BQ769x2Header.h"
#include <math.h>
#include "BQ76952.h"
#include "UV_Recovery.h"

#define NUM_CELLS 4  
#define BALANCE_MIN_DELTA       20           // Minimum voltage delta for balancing (mV)
#define BALANCE_STOP_DELTA      10            // Stop balancing when delta is below this (mV)
#define BALANCE_MIN_VOLTAGE     3100         // Minimum voltage for balancing (mV)

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

Error_Flag error_flag1;

const uint32_t expected_lifespan_cycles = 2000;

#define DEV_ADDR  0x10  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
//#define DEV_ADDR  0x8  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_Mode 0  // 0 for disabled, 1 for enabled
//#define MAX_BUFFER_SIZE 10
#define MAX_BUFFER_SIZE 64
#define R 0 // Read; Used in DirectCommands and Subcommands functions
#define W 1 // Write; Used in DirectCommands and Subcommands functions
#define W2 2 // Write data with two bytes; Used in Subcommands function

uint8_t RX_data[2] = {0x00, 0x00}; // used in several functions to store data read from BQ769x2
uint8_t RX_32Byte [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	//used in Subcommands read function
// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
uint16_t CellVoltage [16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
double temperature [4] = {0,0,0,0};
//float temperature1 [4] = {0,0,0,0};
uint16_t u16Total_Stack_Voltage = 0x00;
uint16_t Battery_Voltage = 0x00;
uint16_t Pack_Voltage = 0x00;
uint16_t LD_Voltage = 0x00;
//uint16_t Pack_Current = 0x00;
//int16_t Pack_Current = 0x00;
float Pack_Current = 0x00;

//New added
uint8_t AFE_Comm_Fault = 0;
uint32_t AFE_I2C_Error_Count = 0;

uint16_t ControlBits = 0x00;
uint16_t AlarmBits = 0x00;
uint16_t RAWAlarmBits = 0x00;
uint16_t Battery_Status = 0x00;
uint8_t value_SafetyStatusA;  // Safety Status Register A
uint8_t value_SafetyStatusB;  // Safety Status Register B
uint8_t value_SafetyStatusC;  // Safety Status Register C
uint8_t value_PFStatusA;   // Permanent Fail Status Register A
uint8_t value_PFStatusB;   // Permanent Fail Status Register B
uint8_t value_PFStatusC;   // Permanent Fail Status Register C
uint8_t FET_Status;  // FET Status register contents  - Shows states of FETs
uint16_t CB_ActiveCells;  // Cell Balancing Active Cells

uint8_t	UV_Fault = 0;   // under-voltage fault state
uint8_t	OV_Fault = 0;   // over-voltage fault state
uint8_t	SCD_Fault = 0;  // short-circuit fault state
uint8_t	SCD_Latch = 0;  // short-circuit fault state
uint8_t	OCD_Fault = 0;  // over-current fault state
uint8_t OT_Fault = 0;// Overtemperature Fault state
uint8_t UT_Fault = 0; // Undertemperature Fault state
uint8_t CellImbalance_Fault = 0; // Cell Imbalance Fault state
uint8_t FET_Fault = 0; // FET Driver Fault state
uint8_t ProtectionsTriggered = 0; // Set to 1 if any protection triggers
uint8_t	Cell_Balancing_State = 0;   // Cell Balancing state
uint8_t	RAW_Cell_Balancing_State = 0;   // Cell Balancing state
uint8_t	Wake_State = 0;   // Wake state
uint8_t	ADC_Scan_Complete = 0;   // ADC Scan Completion State 
uint8_t	Full_Scan_Complete = 0;   // Full Scan Completion State 

uint8_t Sleep_state=0;
uint8_t Sleep_Flag =0;
uint8_t LD_ON = 0;	// Load Detect status bit
uint8_t DSG = 0;   // discharge FET state
uint8_t CHG = 0;   // charge FET state
uint8_t PCHG = 0;  // pre-charge FET state
uint8_t PDSG = 0;  // pre-discharge FET state

uint32_t AccumulatedCharge_Int; // in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Frac;// in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Time;// in BQ769x2_READPASSQ func

uint16_t balance_mask = 0;
uint16_t voltage_delta;
uint8_t max_cell = 0, min_cell = 0;
uint8_t Max_Cell_Voltage_Num, Min_Cell_Voltage_Num, Battery_State;
uint8_t thermal_condition1, thermal_condition2, thermal_condition3, thermal_event;
uint8_t temp_error_flag =0;
uint8_t open_wire_error_flag =0, cell_volt_diff_flag =0;
float u16Discharging_Current;
float Charging_Current;
float SOC_Current_Value;
float soc,soc_prev ;
float coulomb_accumulated;
uint8_t final_soc,soh;
uint16_t Total_cycles;
uint16_t Cycle_Count=0;
float dis_curr_tune;
uint16_t Remaining_Capacity, current_capacity =0;
uint16_t Max_Cell_Voltage, Min_Cell_Voltage, CellVoltDiff;
float Internal_Resistance,Cell_Internal_Resistance[17];
//uint8_t Cell_balancing_state;
uint8_t App_final_chg_current, App_final_dis_current;
uint16_t final_chg_current, final_dis_current,CAN_Current_data;
uint8_t SOC_init_flag =0;
uint8_t Max_Temp, Max_Temp_No, Min_Temp, Min_Temp_No;
uint8_t battery_flag =0;
uint8_t short_revive_counter=0;
uint8_t short_ckt_enabled=0;
uint8_t Short_Ckt_Latch_Counter=0;

//New added
static uint8_t afe_consecutive_failures = 0;
static uint8_t afe_recovery_in_progress = 0;

uint32_t t_prev = 0;
uint32_t t_now = 0;
uint32_t delta_t = 0;

extern  int32_t s32aSystem_Tuning_Variable_Parameters[26];
extern void save_SOC_and_Cycles(uint8_t soc, uint16_t cycles, uint8_t flag);
extern void AFE_Init(void);

#define   chg_current_tunning         s32aSystem_Tuning_Variable_Parameters[20]
#define   dis_chg_current_tunning         s32aSystem_Tuning_Variable_Parameters[21]
#define   balance_start_voltage         s32aSystem_Tuning_Variable_Parameters[11]
#define   Balancing_Delta         s32aSystem_Tuning_Variable_Parameters[24]
#define   battery_capacity         s32aSystem_Tuning_Variable_Parameters[16]
#define CONVERT_TO_COULOMBS(capacity_ah) ((capacity_ah) * 3600) // Convert Ah to Coulombs

#define   dis_chg_high_current_tunning         s32aSystem_Tuning_Variable_Parameters[21]
#define   dis_chg_high_current_tunning1         s32aSystem_Tuning_Variable_Parameters[21]

extern void delay_1us(uint32_t count);

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.	
{
	unsigned char i;
	unsigned char checksum = 0;

	for(i=0; i<len; i++)
		checksum += ptr[i];

	checksum = 0xff & ~checksum;

	return(checksum);
}

unsigned char CRC8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions 
{
	unsigned char i;
	unsigned char crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= 0x107;
			}
			else
				crc *= 2;

			if((*ptr & i)!=0)
				crc ^= 0x107;
		}
		ptr++;
	}
	return(crc);
}

void I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
	uint8_t TX_Buffer [MAX_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#if CRC_Mode
	{
		uint8_t crc_count = 0;
		crc_count = count * 2;
		uint8_t crc1stByteBuffer [3] = {0x10, reg_addr, reg_data[0]};
		unsigned int j;
		unsigned int i;
		uint8_t temp_crc_buffer [3];

		TX_Buffer[0] = reg_data[0];
		TX_Buffer[1] = CRC8(crc1stByteBuffer,3);

		j = 2;
		for(i=1; i<count; i++)
		{
			TX_Buffer[j] = reg_data[i];
			j = j + 1;
			temp_crc_buffer[0] = reg_data[i];
			TX_Buffer[j] = CRC8(temp_crc_buffer,1);
			j = j + 1;
		}
		if (HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg_addr, 1, TX_Buffer, crc_count, 1000) != HAL_OK)
		{
			AFE_I2C_Error_Count++;
			AFE_Comm_Fault = 1;
		}
	}
#else 
	if (HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 1000) != HAL_OK)
	{
		AFE_I2C_Error_Count++;
		AFE_Comm_Fault = 1;
	}
#endif
}

/*
int I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
	unsigned int RX_CRC_Fail = 0;  // reset to 0. If in CRC Mode and CRC fails, this will be incremented.
	uint8_t RX_Buffer [MAX_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#if CRC_Mode
	{
		uint8_t crc_count = 0;
		uint8_t ReceiveBuffer [10] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		crc_count = count * 2;
		unsigned int j;
		unsigned int i;
		unsigned char CRCc = 0;
		uint8_t temp_crc_buffer [3];

		HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, ReceiveBuffer, crc_count, 1000);
		uint8_t crc1stByteBuffer [4] = {0x10, reg_addr, 0x11, ReceiveBuffer[0]};
		CRCc = CRC8(crc1stByteBuffer,4);
		if (CRCc != ReceiveBuffer[1])
		{
			RX_CRC_Fail += 1;
		}
		RX_Buffer[0] = ReceiveBuffer[0];

		j = 2;
		for (i=1; i<count; i++)
		{
			RX_Buffer[i] = ReceiveBuffer[j];
			temp_crc_buffer[0] = ReceiveBuffer[j];
			j = j + 1;
			CRCc = CRC8(temp_crc_buffer,1);
			if (CRCc != ReceiveBuffer[j])
				RX_CRC_Fail += 1;
			j = j + 1;
		}
		CopyArray(RX_Buffer, reg_data, crc_count);
	}
#else
	HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 1000);
#endif
	return 0;
}
*/

int I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    unsigned int RX_CRC_Fail = 0;   // 0 = OK, >0 = CRC error(s)
    uint8_t RX_Buffer[MAX_BUFFER_SIZE] = {0};
    uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};

#if CRC_Mode
    {
        /* -------------------------------------------------------------
         *  CRC is enabled on the BQ7695202 for EVERY read EXCEPT:
         *      0x3C  (CC3 Current)
         *      0x3E,0x3F (sub-command registers)
         *      0x60,0x61 (checksum/length)
         *  For these registers the device returns only the raw data.
         * ------------------------------------------------------------- */
        uint8_t crc_count = count * 2;                 // default: data + CRC
        uint8_t expect_crc = 1;                        // assume we need CRC

        if (reg_addr == 0x3C || reg_addr == 0x3E || reg_addr == 0x3F ||
            reg_addr == 0x60 || reg_addr == 0x61) {
            crc_count = count;        // only data bytes
            expect_crc = 0;
        }

        /* ---------- I2C transfer ---------- */
        HAL_StatusTypeDef hal_stat = HAL_I2C_Mem_Read(
                &hi2c1, DEV_ADDR, reg_addr, 1,
                ReceiveBuffer, crc_count, 1000);

        if (hal_stat != HAL_OK) {
            AFE_I2C_Error_Count++;
            AFE_Comm_Fault = 1;
            return -1;                 // I2C error (timeout, NACK, etc.)
        }

        /* ---------- CRC validation (only if expected) ---------- */
        if (expect_crc) {
            uint8_t j = 0;
            uint8_t CRCc = 0;
            uint8_t temp_crc_buffer[3];

            /* First data byte: CRC over {0x11, reg_addr, 0x11, data[0]} */
            uint8_t crc1stByteBuffer[4] = {0x11, reg_addr, 0x11, ReceiveBuffer[0]};
            CRCc = CRC8(crc1stByteBuffer, 4);
            if (CRCc != ReceiveBuffer[1]) RX_CRC_Fail++;

            RX_Buffer[0] = ReceiveBuffer[0];
            j = 2;

            for (uint8_t i = 1; i < count; i++) {
                RX_Buffer[i] = ReceiveBuffer[j];
                temp_crc_buffer[0] = ReceiveBuffer[j];
                j++;
                CRCc = CRC8(temp_crc_buffer, 1);
                if (CRCc != ReceiveBuffer[j]) RX_CRC_Fail++;
                j++;
            }
        } else {
            /* No CRC – just copy the raw data */
            for (uint8_t i = 0; i < count; i++) {
                RX_Buffer[i] = ReceiveBuffer[i];
            }
        }

        /* Return only the *actual* data bytes to the caller */
        CopyArray(RX_Buffer, reg_data, count);
    }
#else
    /* CRC disabled in software – simple read */
    if (HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 1000) != HAL_OK)
    {
        AFE_I2C_Error_Count++;
        AFE_Comm_Fault = 1;
        return -1;
    }
#endif

    return (int)RX_CRC_Fail;   // 0 = all CRCs OK
}

static int BQ769x2_ReadDirectU16(uint8_t command, uint16_t *value)
{
	uint8_t local_rx[2] = {0, 0};

	if (I2C_ReadReg(command, local_rx, 2) < 0)
	{
		return -1;
	}

	RX_data[0] = local_rx[0];
	RX_data[1] = local_rx[1];
	*value = ((uint16_t)local_rx[1] << 8) | local_rx[0];
	return 0;
}

uint8_t AFE_stuck_flag = 0;

static void BQ769x2_RecoverComm(void)
{
	if (afe_recovery_in_progress)
	{
		return;
	}
  
	AFE_stuck_flag++;
	afe_recovery_in_progress = 1;
	HAL_I2C_DeInit(&hi2c1);
	MX_I2C1_Init();
	HAL_Delay(5);
	AFE_Init();
	AFE_Comm_Fault = 0;
	afe_consecutive_failures = 0;
	afe_recovery_in_progress = 0;
}


void BQ769x2_ServiceCommErrors(void)
{
	if (AFE_Comm_Fault == 0)
	{
		afe_consecutive_failures = 0;
		return;
	}

	if (afe_consecutive_failures < 255)
	{
		afe_consecutive_failures++;
	}

	if (afe_consecutive_failures >= 3)
	{
		BQ769x2_RecoverComm();
	}
}


void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
	uint8_t TX_Buffer[2] = {0x00, 0x00};
	uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	//TX_RegData in little endian format
	TX_RegData[0] = reg_addr & 0xff; 
	TX_RegData[1] = (reg_addr >> 8) & 0xff;
	TX_RegData[2] = reg_data & 0xff; //1st byte of data

	switch(datalen)
    {
		case 1: //1 byte datalength
      I2C_WriteReg(0x3E, TX_RegData, 3);
			delay_1us(2000);
			TX_Buffer[0] = Checksum(TX_RegData, 3); 
			TX_Buffer[1] = 0x05; //combined length of register address and data
      		I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
			delay_1us(2000);
			break;
		case 2: //2 byte datalength
			TX_RegData[3] = (reg_data >> 8) & 0xff;
			I2C_WriteReg(0x3E, TX_RegData, 4);
			delay_1us(2000);
			TX_Buffer[0] = Checksum(TX_RegData, 4); 
			TX_Buffer[1] = 0x06; //combined length of register address and data
      		I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
			delay_1us(2000);
			break;
		case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
			TX_RegData[3] = (reg_data >> 8) & 0xff;
			TX_RegData[4] = (reg_data >> 16) & 0xff;
			TX_RegData[5] = (reg_data >> 24) & 0xff;
			I2C_WriteReg(0x3E, TX_RegData, 6);
			delay_1us(2000);
			TX_Buffer[0] = Checksum(TX_RegData, 6); 
			TX_Buffer[1] = 0x08; //combined length of register address and data
      		I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
			delay_1us(2000);
			break;
    }
}

void CommandSubcommands(uint16_t command) //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{	//For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively
	
	uint8_t TX_Reg[2] = {0x00, 0x00};
	/* Apply GPIO vetoes before a command can release the AFE FET override.
	 * This also covers overcurrent/undervoltage recovery and sleep exit.
	 */
	if ((command == ALL_FETS_ON) || (command == FET_ENABLE))
	{
		OperateChargingMosfet();
		OperateLoadMosfet();
	}

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	I2C_WriteReg(0x3E,TX_Reg,2); 
	delay_1us(2000);
}

void Subcommands(uint16_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
{
	//security keys and Manu_data writes dont work with this function (reading these commands works)
	//max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
	uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t TX_Buffer[2] = {0x00, 0x00};

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff; 

	if (type == R) {//read
		I2C_WriteReg(0x3E,TX_Reg,2);
		delay_1us(2000);
		I2C_ReadReg(0x40, RX_32Byte, 32); //RX_32Byte is a global variable
	}
	else if (type == W) {
		//FET_Control, REG12_Control
		TX_Reg[2] = data & 0xff; 
		I2C_WriteReg(0x3E,TX_Reg,3);
		delay_1us(1000);
		TX_Buffer[0] = Checksum(TX_Reg, 3);
		TX_Buffer[1] = 0x05; //combined length of registers address and data
		I2C_WriteReg(0x60, TX_Buffer, 2);
		delay_1us(1000); 
	}
	else if (type == W2){ //write data with 2 bytes
		//CB_Active_Cells, CB_SET_LVL
		TX_Reg[2] = data & 0xff; 
		TX_Reg[3] = (data >> 8) & 0xff;
		I2C_WriteReg(0x3E,TX_Reg,4);
		delay_1us(1000);
		TX_Buffer[0] = Checksum(TX_Reg, 4); 
		TX_Buffer[1] = 0x06; //combined length of registers address and data
		I2C_WriteReg(0x60, TX_Buffer, 2);
		delay_1us(1000); 
	}
}

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{	//type: R = read, W = write
	uint8_t TX_data[2] = {0x00, 0x00};

	//little endian format
	TX_data[0] = data & 0xff;
	TX_data[1] = (data >> 8) & 0xff;

	if (type == R) {//Read
		RX_data[0] = 0;
		RX_data[1] = 0;
		I2C_ReadReg(command, RX_data, 2); //RX_data is a global variable
		delay_1us(2000);
	}
	if (type == W) {//write
		
    //Control_status, alarm_status, alarm_enable all 2 bytes long
		I2C_WriteReg(command,TX_data,2);
		delay_1us(2000);
	}
}

void BQ769x2_Init() {
	// Configures all parameters in device RAM

	// Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
	// See TRM for full description of CONFIG_UPDATE mode
	CommandSubcommands(SET_CFGUPDATE);

	// After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
	// programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
	// An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
	// a full description of the register and the bits will pop up on the screen.

	// 'Power Config' - 0x9234 = 0x2D80
	// Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
  	// Set wake speed bits to 00 for best performance
	BQ769x2_SetRegister(PowerConfig, 0x2D80, 2);

	// 'REG0 Config' - set REG0_EN bit to enable pre-regulator
	BQ769x2_SetRegister(REG0Config, 0x01, 1);

	// 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
	//BQ769x2_SetRegister(REG12Config, 0xDD, 4);
	BQ769x2_SetRegister(REG12Config, 0x00, 4);
	
	BQ769x2_SetRegister(SleepCurrent, 0x01F4, 2); //500mA
	BQ769x2_SetRegister(SleepHysteresisTime, 0x0F, 2); //15 Sec
	
	
	//BQ769x2_SetRegister(CCGain, 0x418EC3FF, 4);    //0x41B5C28F

	// Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
	//////BQ769x2_SetRegister(DFETOFFPinConfig, 0x42, 1);
	BQ769x2_SetRegister(DFETOFFPinConfig, 0x82, 1);
	BQ769x2_SetRegister(CFETOFFPinConfig, 0x82, 1);

	// Set up ALERT Pin - 0x92FC = 0x2A
	// This configures the ALERT pin to drive high (REG1 voltage) when enabled.
	// The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
	BQ769x2_SetRegister(ALERTPinConfig, 0x2A, 1);

	// Set TS1 to measure Cell Temperature - 0x92FD = 0x07         
	BQ769x2_SetRegister(TS1Config, 0x07, 1);
	
	// Set TS2 to measure Cell Temperature - 0x92FD = 0x07         
	BQ769x2_SetRegister(TS2Config, 0x00, 1);
	
	// Set TS3 to measure FET Temperature - 0x92FF = 0x0F
	//BQ769x2_SetRegister(TS3Config, 0x0F, 1);
	BQ769x2_SetRegister(TS3Config, 0x00, 1); //Cell Temperature

	// Set HDQ to measure Cell Temperature - 0x9300 = 0x07
	BQ769x2_SetRegister(HDQPinConfig, 0x00, 1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

	// 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
	BQ769x2_SetRegister(VCellMode, 0x000F, 2);

	// Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
	// Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
	// COV (over-voltage), CUV (under-voltage)
//	BQ769x2_SetRegister(EnabledProtectionsA, 0xBC, 1);
  BQ769x2_SetRegister(EnabledProtectionsA, 0x80, 1); //ProtectionsA

	// Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
	// Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
	// OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
//
  //BQ769x2_SetRegister(EnabledProtectionsB, 0xF7, 1);
	BQ769x2_SetRegister(EnabledProtectionsB, 0x00, 1); //Disable ProtectionsB
	BQ769x2_SetRegister(EnabledProtectionsC, 0x40, 1); //enable SCDL protection

	// 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
	BQ769x2_SetRegister(DefaultAlarmMask, 0xF882, 2);


  BQ769x2_SetRegister(BalancingConfiguration, 0x0001, 2); //0x09
	BQ769x2_SetRegister(CellBalanceInterval, 0x0002, 2);       // 20s
 
	// Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
	// CUV Threshold is this value multiplied by 50.6mV
	//BQ769x2_SetRegister(CUVThreshold, 0x31, 1);

	// Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
	// COV Threshold is this value multiplied by 50.6mV
 //BQ769x2_SetRegister(COVThreshold, 0x47, 1);  //0x47 = 3592
	
	
	BQ769x2_SetRegister(ShutdownTemperature, 120, 1);    // 115 °C
  BQ769x2_SetRegister(ShutdownTemperatureDelay, 5, 1);      // keep 5 s delay
	
	
	BQ769x2_SetRegister(DAConfiguration, 0x06, 1);      // Current setting
	
	//BQ769x2_SetRegister(0x922D, 15, 1);
	
	//Set overtemperature
	//BQ769x2_SetRegister(OTCThreshold, 0x3C, 1);  // Cell Charge Over Temperature 60 C
	//BQ769x2_SetRegister(OTFThreshold, 0x46, 1);  // Mosfet Over Temperature 70
	//BQ769x2_SetRegister(OTDThreshold, 0x3C, 1);  // //Discharge Cell Over Temperature 60
	
	//BQ769x2_SetRegister(LOAD_DETECT_ON, 0x01, 1);  // Load Detection ON
	
	BQ769x2_SetRegister(FETOptions, 0x1F, 1);  // FET options, Enable predischarge
	BQ769x2_SetRegister(PredischargeTimeout, 0x01F4, 2);  // Timeout //01F4
	BQ769x2_SetRegister(PredischargeStopDelta, 0x0000, 2);  // Stop voltage Delta
	//BQ769x2_SetRegister(PDSGTEST, 0x00, 1);  // Manually control PDSG Mosfet
	
	
//	BQ769x2_SetRegister(0x92B4, 0x0050, 2);  // Set Load Detect Active Time
 // BQ769x2_SetRegister(0x92B5, 0x0010, 2);  // Set Load Detect Retry Delay
 // BQ769x2_SetRegister(0x92B6, 0x0078, 2);  // Set Load Detect Timeout


	// Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
	//BQ769x2_SetRegister(OCCThreshold, 0x05, 1);

	// Set up OCD1 Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
	//BQ769x2_SetRegister(OCD1Threshold, 0x0A, 1);


  BQ769x2_SetRegister(ProtectionConfiguration, 0x0400, 2); //Set the SCDL_CURR_RECOV bit to recover from short circuit when charge current is applied
	BQ769x2_SetRegister(PFAlertMaskB, 0x83, 1);  // Enabled Alert for SCDL
	BQ769x2_SetRegister(DSGFETProtectionsC, 0x40, 1); //Enabled SCDL Protection on DSG
	BQ769x2_SetRegister(CHGFETProtectionsC, 0x00, 1); //Disable SCDL Protection on DSG
	//BQ769x2_SetRegister(EnabledPFB, 0x80, 1); //Enable PF for SCDL

	// Set up SCD Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
	BQ769x2_SetRegister(SCDThreshold, 0x08, 1); //0x05 for 150A //0x07
	// Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 µs; min value of 1    
	//BQ769x2_SetRegister(SCDDelay, 0x1F, 1); //set delay to 500us
	BQ769x2_SetRegister(SCDDelay, 0x10, 1); //set delay to 500us //225uS
	BQ769x2_SetRegister(SCDRecoveryTime, 0x1E, 1); //30 seconds

	// Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
	// If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
	BQ769x2_SetRegister(SCDLLatchLimit, 0x02, 1); 
	BQ769x2_SetRegister(SCDLCounterDecDelay, 0x78, 1); //Protections:SCDL:Counter Dec Delay		120S delay
	BQ769x2_SetRegister(SCDLRecoveryTime, 0x01, 1);  //Protections:SCDL:Recovery Time	
	BQ769x2_SetRegister(SCDLRecoveryThreshold, 0x03E8, 2); //Protections:SCDL:Recovery Threshold	

	// Exit CONFIGUPDATE mode  - Subcommand 0x0092
	CommandSubcommands(EXIT_CFGUPDATE);
}

//  ********************************* FET Control Commands  ***************************************
void DFETOFF()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);  // DFETOFF pin set high
}

void CFETOFF()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);  // CFETOFF pin set high
}

void DFETON()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);  // DFETOFF pin set Low
}

void CFETON()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);  // CFETOFF pin set Low
}


void BQ769x2_BOTHOFF () {
	// Disables all FETs using the DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);  // DFETOFF pin set high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);  // CFETOFF pin set high
}

void BQ769x2_RESET_BOTHOFF () {
	// Resets DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);  // DFETOFF pin set low
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);  // CFETOFF pin set low
}

void BQ769x2_ReadFETStatus() { 
	// Read FET Status to see which FETs are enabled
	DirectCommands(FETStatus, 0x00, R);
	FET_Status = (RX_data[1]*256 + RX_data[0]);
	DSG = ((0x4 & RX_data[0])>>2);// discharge FET state
  CHG = (0x1 & RX_data[0]);// charge FET state
  PCHG = ((0x2 & RX_data[0])>>1);// pre-charge FET state
  PDSG = ((0x8 & RX_data[0])>>3);// pre-discharge FET state
}

void BQ769x2_ReadControlStatus(void) { 
	// Read Control Status for Load Detection
	DirectCommands(ControlStatus, 0x00, R);
	ControlBits = (RX_data[1]*256 + RX_data[0]);
  LD_ON = (0x1 & RX_data[0]);// Load Detection status
}

void BQ769x2_ReadBatteryStatus(void) { 
	// Read Control Status for Load Detection
	uint16_t battery_status = 0;

	if (BQ769x2_ReadDirectU16(BatteryStatus, &battery_status) != 0)
	{
		return;
	}

	Battery_Status = battery_status;
  Sleep_state = (Battery_Status & (1 << 15)) ? 1 : 0;
}


// ********************************* End of FET Control Commands *********************************

// ********************************* BQ769x2 Power Commands   *****************************************

void BQ769x2_ShutdownPin() {
	// Puts the device into SHUTDOWN mode using the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function	
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);  // Sets RST_SHUT pin
}

void BQ769x2_ReleaseShutdownPin() {
	// Releases the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function	
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);  // Resets RST_SHUT pin
}

// ********************************* End of BQ769x2 Power Commands   *****************************************


// ********************************* BQ769x2 Status and Fault Commands   *****************************************

uint16_t BQ769x2_ReadAlarmStatus() { 
	// Read this register to find out why the ALERT pin was asserted
	uint16_t alarm_status = 0;

	if (BQ769x2_ReadDirectU16(AlarmStatus, &alarm_status) != 0)
	{
		return 0;
	}

	return alarm_status;
}

void BQ769x2_ReadAlarmRAWStatus() { 
	// Read this register to find out why the ALERT pin was asserted
	uint16_t raw_alarm_status = 0;

	if (BQ769x2_ReadDirectU16(AlarmRawStatus, &raw_alarm_status) != 0)
	{
		return;
	}

	RAWAlarmBits = raw_alarm_status;
  RAW_Cell_Balancing_State = (RAWAlarmBits & 0x0004) >> 2;
}


void BQ769x2_ReadSafetyStatus() {
    // Read Safety Status A
    DirectCommands(SafetyStatusA, 0x00, R); // Read Safety Status A register
    value_SafetyStatusA = (RX_data[1] << 8) + RX_data[0]; // Combine two bytes into a 16-bit value

    // Extract specific fault flags from Safety Status A
    UV_Fault = (value_SafetyStatusA & 0x0004) >> 2; // Undervoltage Fault
    OV_Fault = (value_SafetyStatusA & 0x0008) >> 3; // Overvoltage Fault
    SCD_Fault = (value_SafetyStatusA & (1 << 7)) >> 7;  // bit 7 // Short Circuit Fault
    OCD_Fault = (value_SafetyStatusA & 0x0002) >> 1; // Overcurrent in Discharge Fault
	
	  if(SCD_Fault==1)
		{
			short_ckt_enabled=1;
		//	LoadState = eLoadOFF; 
			//CharginggState=eChargingOFF;
			error_flag1 =SCD;
			Short_Ckt_Latch_Counter++;
		}
		if(SCD_Fault==0 && short_ckt_enabled==1)
		{
		//	LoadState = eLoadON; 
			//CharginggState=eChargingON;
			short_ckt_enabled=0;
			error_flag1 =NA;
			short_revive_counter=0;
		}
		

    // Read Safety Status B
    DirectCommands(SafetyStatusB, 0x00, R); // Read Safety Status B register
    value_SafetyStatusB = (RX_data[1] << 8) + RX_data[0]; // Combine two bytes into a 16-bit value

    // Extract specific fault flags from Safety Status B
    OT_Fault = (value_SafetyStatusB & 0x0001);      // Overtemperature Fault
    UT_Fault = (value_SafetyStatusB & 0x0002) >> 1; // Undertemperature Fault
    CellImbalance_Fault = (value_SafetyStatusB & 0x0010) >> 4; // Cell Imbalance Fault
    FET_Fault = (value_SafetyStatusB & 0x0080) >> 7; // FET Driver Fault

    // Read Safety Status C
    DirectCommands(SafetyStatusC, 0x00, R); // Read Safety Status C register
    value_SafetyStatusC = (RX_data[1] << 8) + RX_data[0]; // Combine two bytes into a 16-bit value

    // Extract specific fault flags from Safety Status C
   // Internal_IC_Fault = (value_SafetyStatusC & 0x0001);       // Internal IC Fault
   // CC_Fault = (value_SafetyStatusC & 0x0002) >> 1;           // Coulomb Counter Fault
   // Communication_Fault = (value_SafetyStatusC & 0x0010) >> 4; // Communication Fault

    // Check if any protections have been triggered
    if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 0) {
        ProtectionsTriggered = 1; // At least one protection flag is set
    } else {
        ProtectionsTriggered = 0; // No protection flags set
    }
}

void BQ769x2_ReadPFStatus() {
	// Read Permanent Fail Status A/B/C and find which bits are set
	// This shows which permanent failures have been triggered
	DirectCommands(PFStatusA, 0x00, R);
	value_PFStatusA = (RX_data[1]*256 + RX_data[0]);
	DirectCommands(PFStatusB, 0x00, R);
	value_PFStatusB = (RX_data[1]*256 + RX_data[0]);
	SCD_Latch = (value_PFStatusB & (1 << 7)) >> 7;  // bit 7 // Short Circuit Fault
	DirectCommands(PFStatusC, 0x00, R);
	value_PFStatusC = (RX_data[1]*256 + RX_data[0]);
}

// ********************************* End of BQ769x2 Status and Fault Commands   *****************************************


// ********************************* BQ769x2 Measurement Commands   *****************************************


uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
	uint16_t raw_voltage = 0;

	if (BQ769x2_ReadDirectU16(command, &raw_voltage) != 0)
	{
		return 0;
	}

	if(command >= Cell1Voltage && command <= Cell16Voltage) {//Cells 1 through 16 (0x14 to 0x32)
		return raw_voltage; //voltage is reported in mV
	}
	else {//stack, Pack, LD
		return 10 * raw_voltage; //voltage is reported in 0.01V units
	}

}
void BQ769x2_ReadAllVoltages()
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
  int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
  for (int x = 0; x < NUM_CELLS; x++){//Reads all cell voltages
    CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
    cellvoltageholder = cellvoltageholder + 2;
  }
  Battery_Voltage = BQ769x2_ReadVoltage(StackVoltage);
  Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);
  LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);
}

uint16_t BQ769x2_ReadCurrent() 
// Reads PACK current 
{
	for (uint8_t i = 0; i < sizeof(RX_32Byte); i++)
	{
		RX_32Byte[i] = 0;
	}
	
	Subcommands(0x0075, 0x0000, R);  // DASTATUS5()
	if (AFE_Comm_Fault)
	{
		return 0;
	}
	//DirectCommands(CC2Current, 0x00, R);
	//Pack_Current = (RX_data[1]*256 + RX_data[0]);
  // DASTATUS5 bytes 20–21 = CC3 Current (signed 16-bit, userA units)
  Pack_Current = (int16_t)((RX_32Byte[21] << 8) | RX_32Byte[20]);
	
	Charging_Current=0;
	u16Discharging_Current=0;
	final_chg_current=0;
	final_dis_current=0;
	CAN_Current_data=0;
	SOC_Current_Value=0;
	
	//Charge Current
	if(Pack_Current > 0)
	{
		if(Pack_Current>10)
		{
		Charging_Current = ((chg_current_tunning/10.0)*Pack_Current)/1000.0; //78 for 5mohm 4 shunt
		final_chg_current = Charging_Current*10;	
		CAN_Current_data = final_chg_current;
		SOC_Current_Value = (final_chg_current/10.0);
		final_dis_current=0;
		u16Discharging_Current=0;
		Battery_State = 1;
		}
		
		else
		{
		 Charging_Current=0;
		 final_chg_current=0;
		 CAN_Current_data=0;
		 SOC_Current_Value=0;
		}
	}
	
	//Discharge Current
	if(Pack_Current <= 0)
	{
	// if(Pack_Current<-30)
		if(Pack_Current<-5)
	 {
		/**********************/
		if(Pack_Current>=-1600) //15500 for 40A
	  {
		 dis_curr_tune = (float) (dis_chg_current_tunning/10.0); //80 for 5mohm 4 shunt	
		}
		//if(Pack_Current>-1600 && Pack_Current<-2749) 
		if(Pack_Current < -1600 && Pack_Current >= -2750) 
	  {
		 dis_curr_tune = (float) ((dis_chg_high_current_tunning-3)/10.0); //80 for 5mohm 4 shunt	
		}
		
		if(Pack_Current>-2750) 
	  {
		 dis_curr_tune = (float) ((dis_chg_high_current_tunning1-11)/10.0); //80 for 5mohm 4 shunt	
		}
		
		u16Discharging_Current = (dis_curr_tune * Pack_Current)/1000.0;
		u16Discharging_Current = (-u16Discharging_Current);
		final_dis_current = u16Discharging_Current*10;
		CAN_Current_data = final_dis_current;
		SOC_Current_Value = (-final_dis_current/10.0);// 
		final_chg_current=0;
		Charging_Current=0;
		Battery_State = 2;
	 }
	 
	 else
	 {
		u16Discharging_Current=0;	
		final_dis_current=0;
		CAN_Current_data=0;
		SOC_Current_Value=0;
	 }
	}	
	
	if(final_dis_current==0 && final_chg_current==0)
	{
		Battery_State = 0;
	}
	//return (RX_data[1]*256 + RX_data[0]);  // current is reported in mA
}

float BQ769x2_ReadTemperature(uint8_t command) 
{
	uint16_t raw_temperature = 0;

	if (BQ769x2_ReadDirectU16(command, &raw_temperature) != 0)
	{
		return -273.15f;
	}

	return (0.1f * (float)raw_temperature) - 273.15f;  // converts from 0.1K to Celcius
}

void BQ769x2_ReadPassQ(){ // Read Accumulated Charge and Time from DASTATUS6 
	Subcommands(DASTATUS6, 0x00, R);
	AccumulatedCharge_Int = ((RX_32Byte[3]<<24) + (RX_32Byte[2]<<16) + (RX_32Byte[1]<<8) + RX_32Byte[0]); //Bytes 0-3
	AccumulatedCharge_Frac = ((RX_32Byte[7]<<24) + (RX_32Byte[6]<<16) + (RX_32Byte[5]<<8) + RX_32Byte[4]); //Bytes 4-7
	AccumulatedCharge_Time = ((RX_32Byte[11]<<24) + (RX_32Byte[10]<<16) + (RX_32Byte[9]<<8) + RX_32Byte[8]); //Bytes 8-11
}

// ********************************* End of BQ769x2 Measurement Commands   *****************************************

uint8_t balanced_cells[NUM_CELLS]; // 1 = balanced, 0 = not balanced

void Decode_Active_Balancing_Cells(uint16_t mask)
{
    for (uint8_t cell = 0; cell < NUM_CELLS; cell++) {
        if (mask & (1 << cell)) {
            balanced_cells[cell] = 1;
        } else {
            balanced_cells[cell] = 0;
        }
    }
}

void SetCellBalancing(uint16_t balance_mask) {
    uint8_t tx_data[2];
    tx_data[0] = balance_mask & 0xFF;
    tx_data[1] = (balance_mask >> 8) & 0xFF;
	  Subcommands(CB_ACTIVE_CELLS, balance_mask, W2);
	  //Subcommands(CB_ACTIVE_CELLS, 0x00, R);
}

#define BALANCE_INTERVAL_MS  20000 // 20 seconds
//uint8_t balacing_counter=0;

void HostControlledBalancing() {
	  
//	  balacing_counter=1;
	
	  Max_Cell_Voltage = CellVoltage[0];
	  Min_Cell_Voltage = CellVoltage[0];
	  Max_Cell_Voltage_Num = 0;
    Min_Cell_Voltage_Num = 0;
	
//	  static uint32_t lastBalanceTime = 0;
	  uint32_t now = HAL_GetTick();
	
    // Determine max and min cell voltages
    for (int i = 0; i < NUM_CELLS; i++) {
        if (CellVoltage[i] > Max_Cell_Voltage) {
            Max_Cell_Voltage = CellVoltage[i];
            Max_Cell_Voltage_Num = i;
        }
        if (CellVoltage[i] < Min_Cell_Voltage) {
            Min_Cell_Voltage = CellVoltage[i];
            Min_Cell_Voltage_Num = i;
        }
    }
    // Calculate voltage difference
    voltage_delta = Max_Cell_Voltage - Min_Cell_Voltage;
    // Check if balancing is required
		///if ((now - lastBalanceTime) >= BALANCE_INTERVAL_MS) 
		///{
	//	 lastBalanceTime = now;   // update the timestamp
		// balacing_counter++;
     //if (voltage_delta > BALANCE_MIN_DELTA && Max_Cell_Voltage > BALANCE_MIN_VOLTAGE && Charging_Current>2)  //balance_start_voltage
		 if (voltage_delta > Balancing_Delta && Max_Cell_Voltage > balance_start_voltage && Charging_Current>2)
			{
        // Activate balancing for the highest voltage cell
        balance_mask = (1 << Max_Cell_Voltage_Num);
        SetCellBalancing(balance_mask);
      } 
		 else if (voltage_delta < BALANCE_STOP_DELTA) 
		  {
        // Stop balancing if voltage difference is within limits
        SetCellBalancing(0);
      }
		///}
}

/*****************************************************************************************************/
/*Function Name :BQ76952_Read_Voltage_Current_temp()
	Description:Function to read the Voltage, Current and Temperature from Battery Pack
	Caller Function:Task Scheduler
	Author:Ashish Choudhary
	Org:GIL*/
/*****************************************************************************************************/
void BQ76952_Read_Voltage_Current_temp(void)
{
	AlarmBits = BQ769x2_ReadAlarmStatus();
		if (AFE_Comm_Fault)
		{
			return;
		}
		if (AlarmBits & 0x80) {  // Check if FULLSCAN is complete. If set, new measurements are available
      		BQ769x2_ReadAllVoltages();
			if (AFE_Comm_Fault)
			{
				return;
			}
      		//Pack_Current = BQ769x2_ReadCurrent();
			    BQ769x2_ReadCurrent();
				if (AFE_Comm_Fault)
				{
					return;
				}
			    UV_Recovery_Service();
                BQ769x2_ReadSafetyStatus();
				if (AFE_Comm_Fault)
				{
					return;
				}
      		temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
				if (AFE_Comm_Fault)
				{
					return;
				}
			   // temperature[1] = BQ769x2_ReadTemperature(TS2Temperature);
      		//temperature[2] = BQ769x2_ReadTemperature(TS3Temperature);
			   // temperature[3] = BQ769x2_ReadTemperature(HDQTemperature);
			
			if(temperature[0]>=30 && temperature[0]<75)
			{
			 temperature[0] = 0.88*temperature[0]; //Based on accuracy equation
			}
			if(temperature[0]>=75)
			{
			 temperature[0] = 0.8517*temperature[0];
			}
			
			    Max_Temp = temperature[0];
	        Min_Temp = temperature[0];
					
			 
			    u16Total_Stack_Voltage =0;
			    for (int i = 0; i < NUM_CELLS; i++)   // 15 cells (0…14)
          {
           u16Total_Stack_Voltage += CellVoltage[i];
          }
			
          for(int i=0;i<4;i++)
          {
           if(temperature[i]>Max_Temp)
	         {
            Max_Temp=temperature[i];
		        Max_Temp_No = i;
		       }
          if(temperature[i]<Min_Temp)
	        {
           Min_Temp=temperature[i];
				   Min_Temp_No = i;
		      }
       }	
			
			DirectCommands(AlarmStatus, 0x0080, W);  // Clear the FULLSCAN bit
		}
				
		if (AlarmBits & 0xC000) {  // If Safety Status bits are showing in AlarmStatus register
			UV_Recovery_Service();
                BQ769x2_ReadSafetyStatus(); // Read the Safety Status registers to find which protections have triggered
			if (ProtectionsTriggered & 1) {
			//	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET); }// Turn on the LED to indicate Protection has triggered
				DirectCommands(AlarmStatus, 0xF800, W); // Clear the Safety Status Alarm bits.
			}
		else
		{
			if (ProtectionsTriggered & 1) {
				//UV_Recovery_Service();
                BQ769x2_ReadSafetyStatus();
				if (!(ProtectionsTriggered & 1)) 
				{
					//HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
				} 
			} // Turn off the LED if Safety Status has cleared which means the protection condition is no longer present
		}
		//delay_1us(20000);  // repeat loop every 20 ms
  }
	}

	
// Parameters
float Energy_Delivered_last_Cycle_kWh = 0;
float Total_Charge_Ah = 0;
float Total_Discharge_Ah = 0;
float Pre_Discharge_Ah = 0;
//float DischargeAH_Diff =0;
float Total_Energy_kWh = 0;
float BackupTime_Hours = 0;
float Remaining_Energy_kWh = 0;
float Total_Charging_kWh = 0;
float Total_Discharging_kWh = 0;
float Power_W;
float DeltaTime = 60.0; // 60 seconds
float Energy_kWh;
float Load_Power;
float ChargeTime_Hours = 0;
float Nominal_Battery_Voltage = (3.2f*NUM_CELLS);

uint8_t hours, minutes;
uint16_t BackupTime_Hours1=0,BackupTime_minutes=0;

void SOC_Initialization()
{
	soc_prev=0.50;
	//final_soc =50;
	soc =0.50;
	SOC_init_flag =1;
	save_SOC_and_Cycles(final_soc, Total_cycles, SOC_init_flag);
}


#define CAL_DEBOUNCE_MS       5000U
uint32_t high_cal_start = 0;
uint32_t low_cal_start = 0;

static void SOC_CycleBasedCalibration(void)
{
    float pack_voltage = u16Total_Stack_Voltage / 1000.0f;

    if (Total_cycles > 1)
        return;

    // High-side correction during charging
    if ((u16Total_Stack_Voltage >= 14100) && (Charging_Current > 2))
    {
        if (high_cal_start == 0)
            high_cal_start = HAL_GetTick();

        if ((HAL_GetTick() - high_cal_start) >= CAL_DEBOUNCE_MS)
        {
            soc = 1;
			      soc_prev =1;
            final_soc = 100;

            save_SOC_and_Cycles(final_soc, Total_cycles, SOC_init_flag);
            high_cal_start = 0;
            low_cal_start = 0;
        }
    }
    else
    {
        high_cal_start = 0;
    }

    // Low-side correction during discharging
    if ((u16Total_Stack_Voltage <= 12000) && (u16Discharging_Current > 2))
    {
        if (low_cal_start == 0)
            low_cal_start = HAL_GetTick();

        if ((HAL_GetTick() - low_cal_start) >= CAL_DEBOUNCE_MS)
        {
            soc = 0;
            soc_prev = soc;
            final_soc = 0.0f;

            save_SOC_and_Cycles(final_soc, Total_cycles, SOC_init_flag);
            high_cal_start = 0;
            low_cal_start = 0;
        }
    }
    else
    {
        low_cal_start = 0;
    }
}


void SOC_Estimation(void)
{
    t_now = HAL_GetTick();
    delta_t = t_now - t_prev;

    if (SOC_init_flag == 0)
    {
        SOC_Initialization();
        t_prev = t_now;   // prevent large first delta
    }

    Power_W = (u16Total_Stack_Voltage / 1000.0f) * SOC_Current_Value;

    if (delta_t >= 10000U)   // 10 second update
    {
        float delta_hours = delta_t / 3600000.0f;

        t_prev = t_now;

        soc = soc_prev + (SOC_Current_Value * delta_hours) / battery_capacity;

        if (soc > 1.0f) soc = 1.0f;
        if (soc < 0.0f) soc = 0.0f;

        soc_prev = soc;
        final_soc = soc * 100.0f;

        Remaining_Capacity = (final_soc * battery_capacity) / 100.0f;

        Energy_kWh = (Power_W * delta_t) / 3600000000.0f;

        if (SOC_Current_Value > 0)
        {
            Total_Charge_Ah += (SOC_Current_Value * delta_t) / 3600000.0f;
            Total_Charging_kWh += Energy_kWh;
            Energy_Delivered_last_Cycle_kWh = 0;
            BackupTime_Hours = 0;
            BackupTime_minutes = 0;
            Remaining_Energy_kWh = 0;
        }
        else if (SOC_Current_Value < 0)
        {
            Total_Discharge_Ah += (fabs(SOC_Current_Value) * delta_t) / 3600000.0f;
            Total_Discharging_kWh += fabs(Energy_kWh);
            Energy_Delivered_last_Cycle_kWh += fabs(Energy_kWh);

            Remaining_Energy_kWh =
                (final_soc * battery_capacity * Nominal_Battery_Voltage) / (1000.0f * 100.0f);

            Load_Power = fabs(Power_W);

            if (Load_Power > 0.1f)
            {
                BackupTime_Hours = Remaining_Energy_kWh / (Load_Power / 1000.0f);
                hours = (uint16_t)BackupTime_Hours;
                minutes = (uint16_t)((BackupTime_Hours - hours) * 60.0f);
                BackupTime_Hours1 = (uint16_t)(hours * 100 + minutes);
                BackupTime_minutes = (hours * 60) + minutes;
            }
        }
        else
        {
            BackupTime_Hours = 0;
            hours = 0;
            minutes = 0;
            BackupTime_Hours1 = 0;
            BackupTime_minutes = 0;
        }

        Total_Energy_kWh = Total_Charging_kWh + Total_Discharging_kWh;

        if (Charging_Current > 0.0f)
        {
            ChargeTime_Hours = ((100.0f - final_soc) / 100.0f) * battery_capacity / Charging_Current;
        }
        else
        {
            ChargeTime_Hours = 0.0f;
        }

        if ((Total_Discharge_Ah - Pre_Discharge_Ah) >= (battery_capacity-7))
        {
            Total_cycles = Total_cycles + 1;
            Pre_Discharge_Ah = Total_Discharge_Ah;
        }
    }
		SOC_CycleBasedCalibration();
}
	
	
void SOH_Estimation(void)
{
	if (Total_cycles >= expected_lifespan_cycles) {
       // return 80.0f; // Assuming SoH doesn't decrease below 80% in this simplistic model
    }
    // Linear approximation of SoH
  //  soh = 100.0f - (Total_cycles * (20.0f / expected_lifespan_cycles));
		float soh_f = 100.0f - (Total_cycles * (20.0f / expected_lifespan_cycles));
		if (soh_f < 80.0f)
        soh_f = 80.0f;
    else if (soh_f > 100.0f)
        soh_f = 100.0f;
		soh = (uint8_t)(soh_f + 0.5f);
   // return soh > 80.0f ? soh : 80.0f; // Ensure SoH doesn't drop below 80%
}



void Pre_Discharge(void)
{
	if(Sleep_state==1)
	{
	 Sleep_Flag =1;
	}	
  if(Sleep_Flag==1 && Sleep_state==0)
	{
		CommandSubcommands(DSG_PDSG_OFF);
		CommandSubcommands(ALL_FETS_ON);
		Sleep_Flag=0;
	}
}


