#include "debug_io.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *s_debug_uart = NULL;
volatile uint8_t g_debug_trace_enabled = 1U;

void Debug_Init(UART_HandleTypeDef *huart)
{
  s_debug_uart = huart;
  Debug_UserLedSet(0U);
}

void Debug_SetTraceEnabled(uint8_t enabled)
{
  g_debug_trace_enabled = (uint8_t)((enabled != 0U) ? 1U : 0U);
}

uint8_t Debug_IsTraceEnabled(void)
{
  return g_debug_trace_enabled;
}

void Debug_Trace(const char *fmt, ...)
{
  char payload[96];
  char tx_buf[128];
  va_list args;
  int payload_len;
  int tx_len;

  if ((s_debug_uart == NULL) || (fmt == NULL) || (g_debug_trace_enabled == 0U))
  {
    return;
  }

  va_start(args, fmt);
  payload_len = vsnprintf(payload, sizeof(payload), fmt, args);
  va_end(args);

  if ((payload_len <= 0) || ((size_t)payload_len >= sizeof(payload)))
  {
    return;
  }

  tx_len = snprintf(tx_buf, sizeof(tx_buf), "[#DBG:%s]\n", payload);
  if ((tx_len <= 0) || ((size_t)tx_len >= sizeof(tx_buf)))
  {
    return;
  }

  (void)HAL_UART_Transmit(s_debug_uart, (uint8_t *)tx_buf, (uint16_t)tx_len, 50U);
}

void Debug_UserLedSet(uint8_t on)
{
  HAL_GPIO_WritePin(UserLED_GPIO_Port,
                    UserLED_Pin,
                    (on != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Debug_UserLedToggle(void)
{
  HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
}
