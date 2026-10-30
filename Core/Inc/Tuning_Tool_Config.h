#ifndef __TUNING_TOOL_CONFIG__

#define __TUNING_TOOL_CONFIG__

#include "stdint.h"
#define PROGRAM_HALFWORD   0xBB		//(0x01U)  /*!<Program a half-word (16-bit) at a specified address.*/
#define PROGRAM_WORD       0xCC		//(0x02U)  /*!<Program a word (32-bit) at a specified address.*/
#define PROGRAM_DOUBLEWORD 0xDD		//(0x03U)  /*!<Program a double word (64-bit) at a specified address*/


#define FLASH_FKEY1                          ((uint32_t)0x45670123)        /*!< Flash program erase key1 */
#define FLASH_FKEY2                          ((uint32_t)0xCDEF89AB)        /*!< Flash program erase key2: used with FLASH_PEKEY1
                                                                                to unlock the write access to the FPEC. */
#define IS_FLASH_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFFFFFFCB) == 0x00000000) && ((FLAG) != 0x00000000))

extern uint8_t Flash_Erase_and_Program(void);
extern uint32_t UART_Tuning_Tool_Callback(uint32_t u32rx_char);
extern void Copy_Flash_To_Ram(int32_t *s32pram_address, int64_t *s32pflash_address, uint8_t u8size);

#endif

