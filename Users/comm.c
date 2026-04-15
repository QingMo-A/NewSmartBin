#include "comm.h"

#include "debug_io.h"

#include <string.h>
#include <stdio.h>

#define COMM_RX_LINE_MAX_LEN 64U
#define COMM_RX_QUEUE_DEPTH  4U

static UART_HandleTypeDef *s_comm_uart = NULL;
static Comm_CommandHandler_t s_command_handler = NULL;
static uint8_t s_rx_byte = 0U;
static char s_line_buffer[COMM_RX_LINE_MAX_LEN];
static volatile uint16_t s_line_index = 0U;
static volatile uint8_t s_line_overflow = 0U;
static char s_line_queue[COMM_RX_QUEUE_DEPTH][COMM_RX_LINE_MAX_LEN];
static volatile uint8_t s_queue_head = 0U;
static volatile uint8_t s_queue_tail = 0U;
static volatile uint8_t s_queue_count = 0U;
static volatile uint8_t s_pending_err = 0U;

static void Comm_StartReceiveIT(void)
{
  if (s_comm_uart != NULL)
  {
    (void)HAL_UART_Receive_IT(s_comm_uart, &s_rx_byte, 1U);
  }
}

static void Comm_ResetCurrentLine(void)
{
  s_line_index = 0U;
  s_line_overflow = 0U;
  memset(s_line_buffer, 0, sizeof(s_line_buffer));
}

static void Comm_QueueLine(const char *line)
{
  if (line == NULL)
  {
    return;
  }

  if (s_queue_count >= COMM_RX_QUEUE_DEPTH)
  {
    s_pending_err = 1U;
    DEBUG_PRINT("RX queue_overflow");
    return;
  }

  (void)snprintf(s_line_queue[s_queue_tail], COMM_RX_LINE_MAX_LEN, "%s", line);
  s_queue_tail = (uint8_t)((s_queue_tail + 1U) % COMM_RX_QUEUE_DEPTH);
  ++s_queue_count;
}

void Comm_Init(UART_HandleTypeDef *huart, Comm_CommandHandler_t handler)
{
  s_comm_uart = huart;
  s_command_handler = handler;
  s_queue_head = 0U;
  s_queue_tail = 0U;
  s_queue_count = 0U;
  s_pending_err = 0U;
  Comm_ResetCurrentLine();
  memset(s_line_queue, 0, sizeof(s_line_queue));
  DEBUG_PRINT("COMM init");
  Comm_StartReceiveIT();
}

void Comm_Task(void)
{
  if (s_pending_err != 0U)
  {
    s_pending_err = 0U;
    DEBUG_PRINT("RX line_error");
    (void)Comm_SendLine(COMM_TX_ERR);
  }

  while (s_queue_count > 0U)
  {
    char line[COMM_RX_LINE_MAX_LEN];
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    (void)snprintf(line, sizeof(line), "%s", s_line_queue[s_queue_head]);
    s_line_queue[s_queue_head][0] = '\0';
    s_queue_head = (uint8_t)((s_queue_head + 1U) % COMM_RX_QUEUE_DEPTH);
    --s_queue_count;
    if (primask == 0U)
    {
      __enable_irq();
    }

    Comm_ProcessLine(line);
  }
}

UART_HandleTypeDef *Comm_GetUartHandle(void)
{
  return s_comm_uart;
}

HAL_StatusTypeDef Comm_SendLine(const char *line)
{
  char tx_buf[96];
  int len;

  if ((s_comm_uart == NULL) || (line == NULL))
  {
    return HAL_ERROR;
  }

  len = snprintf(tx_buf, sizeof(tx_buf), "%s\n", line);
  if ((len <= 0) || ((size_t)len >= sizeof(tx_buf)))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(s_comm_uart, (uint8_t *)tx_buf, (uint16_t)len, 50U);
}

void Comm_ProcessLine(const char *line)
{
  Comm_Command_t cmd;
  size_t len;

  if (line == NULL)
  {
    return;
  }

  len = strlen(line);
  if (len == 0U)
  {
    return;
  }

  DEBUG_PRINT("RX=%s", line);

  cmd.type = COMM_CMD_INVALID;
  cmd.bin_id = 0U;
  cmd.raw_line = line;

  if (strcmp(line, "[$PING]") == 0)
  {
    cmd.type = COMM_CMD_PING;
  }
  else if (strcmp(line, "[$RESET]") == 0)
  {
    cmd.type = COMM_CMD_RESET;
  }
  else if ((strncmp(line, "[$TARGET_BIN:", 13) == 0) &&
           (len == 15U) &&
           (line[13] >= '1') && (line[13] <= '4') &&
           (line[14] == ']'))
  {
    cmd.type = COMM_CMD_TARGET_BIN;
    cmd.bin_id = (uint8_t)(line[13] - '0');
  }

  if (cmd.type == COMM_CMD_INVALID)
  {
    DEBUG_PRINT("RX invalid_cmd");
    (void)Comm_SendLine(COMM_TX_ERR);
    return;
  }

  if (s_command_handler != NULL)
  {
    s_command_handler(&cmd);
  }
}

void Comm_RxByteCallback(UART_HandleTypeDef *huart)
{
  if (huart != s_comm_uart)
  {
    return;
  }

  if (s_rx_byte == '\r')
  {
    Comm_StartReceiveIT();
    return;
  }

  if (s_rx_byte == '\n')
  {
    if (s_line_overflow != 0U)
    {
      s_pending_err = 1U;
    }
    else if (s_line_index > 0U)
    {
      s_line_buffer[s_line_index] = '\0';
      Comm_QueueLine(s_line_buffer);
    }

    Comm_ResetCurrentLine();
    Comm_StartReceiveIT();
    return;
  }

  if (s_line_overflow == 0U)
  {
    if (s_line_index < (COMM_RX_LINE_MAX_LEN - 1U))
    {
      s_line_buffer[s_line_index++] = (char)s_rx_byte;
    }
    else
    {
      s_line_overflow = 1U;
      s_line_index = 0U;
      memset(s_line_buffer, 0, sizeof(s_line_buffer));
    }
  }

  Comm_StartReceiveIT();
}

void Comm_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != s_comm_uart)
  {
    return;
  }

  DEBUG_PRINT("UART error");
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);
  Comm_StartReceiveIT();
}
