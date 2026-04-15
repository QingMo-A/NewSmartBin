#ifndef DEBUG_IO_H
#define DEBUG_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern volatile uint8_t g_debug_trace_enabled;

void Debug_Init(UART_HandleTypeDef *huart);
void Debug_SetTraceEnabled(uint8_t enabled);
uint8_t Debug_IsTraceEnabled(void);
void Debug_Trace(const char *fmt, ...);
void Debug_UserLedSet(uint8_t on);
void Debug_UserLedToggle(void);

#define DEBUG_PRINT(...) Debug_Trace(__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_IO_H */
