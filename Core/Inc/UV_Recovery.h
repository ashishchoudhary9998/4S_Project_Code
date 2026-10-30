#ifndef UV_RECOVERY_H
#define UV_RECOVERY_H
#include <stdint.h>
#define UV_RESET_INDEX 25U
#define UV_RESET_VALUE 0x1DE6U
#define UV_RESET_ADDRESS 0x0FE0U
#define UV_RECOVERY_MS 60000UL
extern volatile uint8_t uv_reset_requested;
void UV_Recovery_Init(void);
void UV_Recovery_Service(void);
#endif