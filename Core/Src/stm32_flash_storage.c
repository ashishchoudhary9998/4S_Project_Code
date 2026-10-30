#include "stm32g0xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include "BQ76952.h"

#define FLASH_USER_START_ADDR  ((uint32_t)0x0800F800)  // Start of the last page (Page 63)
#define PAGE_SIZE              ((uint32_t)0x800)      // 2KB page size
#define MAX_ENTRIES            (PAGE_SIZE / 8)        // 8 bytes per entry (Double word)

// Thresholds for saving
#define SOC_CHANGE_THRESHOLD   3
#define CYCLE_CHANGE_THRESHOLD 1

//extern uint8_t current_SOC;
//extern uint16_t current_Cycles;

static uint32_t get_valid_entry_address() {
    uint64_t *flash_data = (uint64_t *)FLASH_USER_START_ADDR;
    for (int i = 0; i < MAX_ENTRIES; i++) {
        if (flash_data[i] == 0xFFFFFFFFFFFFFFFF) {
            return FLASH_USER_START_ADDR + i * 8;
        }
    }
    return 0xFFFFFFFF; // No empty space
}

static void erase_flash_page() {
    FLASH_EraseInitTypeDef erase_config;
    uint32_t page_error;

    erase_config.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_config.Page = 63;
    erase_config.NbPages = 1;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase_config, &page_error);
    HAL_FLASH_Lock();
}


uint8_t write_Check=0;

void save_SOC_and_Cycles(uint8_t soc, uint16_t cycles, uint8_t flag) {
   // static uint8_t last_soc = 0xFF;
   // static uint16_t last_cycles = 0xFFFF;

    //if (abs((int)final_soc - (int)last_soc) < SOC_CHANGE_THRESHOLD &&
      //  abs((int)Total_cycles - (int)last_cycles) < CYCLE_CHANGE_THRESHOLD) {
      //  return;
    //}

    uint32_t address = get_valid_entry_address();
    if (address == 0xFFFFFFFF) {
        erase_flash_page();
        address = FLASH_USER_START_ADDR;
    }

    uint64_t data = 0;
    data |= soc;
    data |= ((uint64_t)cycles << 8);
		data |= ((uint64_t)flag << 24);  // Store flag in byte 3

    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    HAL_FLASH_Lock();
		
		write_Check++;

    //last_soc = final_soc;
   // last_cycles = Total_cycles;
}

void read_last_SOC_and_Cycles(uint8_t *soc, uint16_t *cycles, uint8_t *flag) {
    uint64_t *flash_data = (uint64_t *)FLASH_USER_START_ADDR;
    for (int i = MAX_ENTRIES - 1; i >= 0; i--) {
        if (flash_data[i] != 0xFFFFFFFFFFFFFFFF) {
            *soc = flash_data[i] & 0xFF;
            *cycles = (flash_data[i] >> 8) & 0xFFFF;
					  *flag = (flash_data[i] >> 24) & 0xFF;
            return;
        }
    }
    *soc = 0;
    *cycles = 0;
		*flag = 0;
}

uint8_t previous_SOC = 0xFF;         // Invalid initial value
uint16_t previous_Cycles = 0xFFFF;

void check_and_store_SOC_Cycles(uint8_t soc, uint16_t cycles, uint8_t flag)
{
    int8_t soc_diff = abs((int16_t)soc - previous_SOC);
    uint16_t cycles_diff = abs((int16_t)cycles - previous_Cycles);

    if ((soc_diff >= 3) || (cycles_diff >= 1)) {
        save_SOC_and_Cycles(soc, cycles, flag);
        previous_SOC = soc;
        previous_Cycles = cycles;
    }
}

