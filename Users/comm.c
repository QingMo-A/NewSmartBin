#include "comm.h"

#include "debug_io.h"

#include <string.h>
#include <stdio.h>

#define COMM_RX_LINE_MAX_LEN 128U
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

static void Comm_ResetRxBuffers(void)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();

  s_rx_byte = 0U;
  s_queue_head = 0U;
  s_queue_tail = 0U;
  s_queue_count = 0U;
  s_pending_err = 0U;
  memset(s_line_queue, 0, sizeof(s_line_queue));
  memset(s_line_buffer, 0, sizeof(s_line_buffer));
  s_line_index = 0U;
  s_line_overflow = 0U;

  if (primask == 0U)
  {
    __enable_irq();
  }
}

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

static void Comm_RecoverUart(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  Comm_ResetRxBuffers();

  __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->pRxBuffPtr = NULL;
  huart->RxXferSize = 0U;
  huart->RxXferCount = 0U;
  huart->RxISR = NULL;
  huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
  huart->RxState = HAL_UART_STATE_READY;

  Comm_StartReceiveIT();
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

static uint8_t Comm_ParseDirection(const char *token, Comm_Direction_t *direction)
{
  if ((token == NULL) || (direction == NULL))
  {
    return 0U;
  }

  if (strcmp(token, "LEFT") == 0)
  {
    *direction = COMM_DIR_LEFT;
    return 1U;
  }

  if (strcmp(token, "RIGHT") == 0)
  {
    *direction = COMM_DIR_RIGHT;
    return 1U;
  }

  return 0U;
}

static uint8_t Comm_IsByteValueValid(unsigned int value)
{
  return (uint8_t)(value <= 255U);
}

static uint8_t Comm_ParseTargetLine(const char *line, Comm_Command_t *cmd)
{
  unsigned int bin_id;
  unsigned int r;
  unsigned int g;
  unsigned int b;
  unsigned int tol_r;
  unsigned int tol_g;
  unsigned int tol_b;
  char dir_buf[8];
  int parsed_len;

  if ((line == NULL) || (cmd == NULL))
  {
    return 0U;
  }

  parsed_len = 0;
  if (sscanf(line,
             "[$TARGET_BIN:%u,DIR:%7[^,],RGB:%u,%u,%u,TOL:%u,%u,%u]%n",
             &bin_id,
             dir_buf,
             &r,
             &g,
             &b,
             &tol_r,
             &tol_g,
             &tol_b,
             &parsed_len) != 8)
  {
    return 0U;
  }

  if ((parsed_len <= 0) || (line[parsed_len] != '\0'))
  {
    return 0U;
  }

  if ((bin_id < 1U) || (bin_id > 4U))
  {
    return 0U;
  }

  if ((Comm_IsByteValueValid(r) == 0U) ||
      (Comm_IsByteValueValid(g) == 0U) ||
      (Comm_IsByteValueValid(b) == 0U) ||
      (Comm_IsByteValueValid(tol_r) == 0U) ||
      (Comm_IsByteValueValid(tol_g) == 0U) ||
      (Comm_IsByteValueValid(tol_b) == 0U))
  {
    return 0U;
  }

  if (Comm_ParseDirection(dir_buf, &cmd->direction) == 0U)
  {
    return 0U;
  }

  cmd->type = COMM_CMD_TARGET_BIN;
  cmd->bin_id = (uint8_t)bin_id;
  cmd->color_spec.r = (uint8_t)r;
  cmd->color_spec.g = (uint8_t)g;
  cmd->color_spec.b = (uint8_t)b;
  cmd->color_spec.tol_r = (uint8_t)tol_r;
  cmd->color_spec.tol_g = (uint8_t)tol_g;
  cmd->color_spec.tol_b = (uint8_t)tol_b;
  return 1U;
}

static uint8_t Comm_ParseHomeColorLine(const char *line, Comm_Command_t *cmd)
{
  unsigned int r;
  unsigned int g;
  unsigned int b;
  unsigned int tol_r;
  unsigned int tol_g;
  unsigned int tol_b;
  int parsed_len;

  if ((line == NULL) || (cmd == NULL))
  {
    return 0U;
  }

  parsed_len = 0;
  if (sscanf(line,
             "[$HOME:RGB:%u,%u,%u,TOL:%u,%u,%u]%n",
             &r,
             &g,
             &b,
             &tol_r,
             &tol_g,
             &tol_b,
             &parsed_len) != 6)
  {
    return 0U;
  }

  if ((parsed_len <= 0) || (line[parsed_len] != '\0'))
  {
    return 0U;
  }

  if ((Comm_IsByteValueValid(r) == 0U) ||
      (Comm_IsByteValueValid(g) == 0U) ||
      (Comm_IsByteValueValid(b) == 0U) ||
      (Comm_IsByteValueValid(tol_r) == 0U) ||
      (Comm_IsByteValueValid(tol_g) == 0U) ||
      (Comm_IsByteValueValid(tol_b) == 0U))
  {
    return 0U;
  }

  cmd->type = COMM_CMD_HOME_COLOR;
  cmd->color_spec.r = (uint8_t)r;
  cmd->color_spec.g = (uint8_t)g;
  cmd->color_spec.b = (uint8_t)b;
  cmd->color_spec.tol_r = (uint8_t)tol_r;
  cmd->color_spec.tol_g = (uint8_t)tol_g;
  cmd->color_spec.tol_b = (uint8_t)tol_b;
  return 1U;
}

static uint8_t Comm_SetConfigValue(const char *key, unsigned int value, App_Config_t *config, uint32_t *mask)
{
  if ((key == NULL) || (config == NULL) || (mask == NULL))
  {
    return 0U;
  }

  if (strcmp(key, "MT") == 0)
  {
    config->move_timeout_ms = (uint32_t)value;
    *mask |= APP_CONFIG_MASK_MOVE_TIMEOUT_MS;
  }
  else if (strcmp(key, "PD") == 0)
  {
    config->preopen_delay_ms = (uint32_t)value;
    *mask |= APP_CONFIG_MASK_PREOPEN_DELAY_MS;
  }
  else if (strcmp(key, "GT") == 0)
  {
    config->gate_timeout_ms = (uint32_t)value;
    *mask |= APP_CONFIG_MASK_GATE_TIMEOUT_MS;
  }
  else if (strcmp(key, "DW") == 0)
  {
    config->drop_wait_ms = (uint32_t)value;
    *mask |= APP_CONFIG_MASK_DROP_WAIT_MS;
  }
  else if (strcmp(key, "PC") == 0)
  {
    config->postclose_delay_ms = (uint32_t)value;
    *mask |= APP_CONFIG_MASK_POSTCLOSE_DELAY_MS;
  }
  else if (strcmp(key, "RH") == 0)
  {
    config->return_home_timeout_ms = (uint32_t)value;
    *mask |= APP_CONFIG_MASK_RETURN_HOME_TIMEOUT_MS;
  }
  else if (strcmp(key, "EB") == 0)
  {
    config->error_blink_ms = (uint32_t)value;
    *mask |= APP_CONFIG_MASK_ERROR_BLINK_MS;
  }
  else if (strcmp(key, "SP") == 0)
  {
    if (value > 255U)
    {
      return 0U;
    }
    config->motion_speed = (uint8_t)value;
    *mask |= APP_CONFIG_MASK_MOTION_SPEED;
  }
  else if (strcmp(key, "OA") == 0)
  {
    if (value > 255U)
    {
      return 0U;
    }
    config->gate_open_angle = (uint8_t)value;
    *mask |= APP_CONFIG_MASK_GATE_OPEN_ANGLE;
  }
  else if (strcmp(key, "CA") == 0)
  {
    if (value > 255U)
    {
      return 0U;
    }
    config->gate_close_angle = (uint8_t)value;
    *mask |= APP_CONFIG_MASK_GATE_CLOSE_ANGLE;
  }
  else if (strcmp(key, "OS") == 0)
  {
    config->gate_open_settle_ms = (uint32_t)value;
    *mask |= APP_CONFIG_MASK_GATE_OPEN_SETTLE_MS;
  }
  else if (strcmp(key, "CS") == 0)
  {
    config->gate_close_settle_ms = (uint32_t)value;
    *mask |= APP_CONFIG_MASK_GATE_CLOSE_SETTLE_MS;
  }
  else
  {
    return 0U;
  }

  return 1U;
}

static uint8_t Comm_ParseConfigLine(const char *line, Comm_Command_t *cmd)
{
  static const char prefix[] = "[$CFG:";
  char body[COMM_RX_LINE_MAX_LEN];
  size_t line_len;
  size_t body_len;
  char *token;

  if ((line == NULL) || (cmd == NULL))
  {
    return 0U;
  }

  line_len = strlen(line);
  if ((line_len <= (sizeof(prefix) - 1U + 1U)) ||
      (strncmp(line, prefix, sizeof(prefix) - 1U) != 0) ||
      (line[line_len - 1U] != ']'))
  {
    return 0U;
  }

  body_len = line_len - (sizeof(prefix) - 1U) - 1U;
  if ((body_len == 0U) || (body_len >= sizeof(body)))
  {
    return 0U;
  }

  (void)memset(body, 0, sizeof(body));
  (void)memcpy(body, &line[sizeof(prefix) - 1U], body_len);

  cmd->config = *AppConfig_Get();
  cmd->config_mask = 0UL;
  token = strtok(body, ",");
  while (token != NULL)
  {
    char key[8];
    unsigned int value;
    int parsed_len = 0;

    if ((sscanf(token, "%7[^=]=%u%n", key, &value, &parsed_len) != 2) ||
        (parsed_len <= 0) ||
        (token[parsed_len] != '\0'))
    {
      return 0U;
    }

    if (Comm_SetConfigValue(key, value, &cmd->config, &cmd->config_mask) == 0U)
    {
      return 0U;
    }

    token = strtok(NULL, ",");
  }

  if (cmd->config_mask == 0UL)
  {
    return 0U;
  }

  cmd->type = COMM_CMD_CONFIG;
  return 1U;
}

void Comm_Init(UART_HandleTypeDef *huart, Comm_CommandHandler_t handler)
{
  s_comm_uart = huart;
  s_command_handler = handler;
  Comm_ResetRxBuffers();
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
  char tx_buf[192];
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

  return HAL_UART_Transmit(s_comm_uart, (uint8_t *)tx_buf, (uint16_t)len, 100U);
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
  cmd.direction = COMM_DIR_UNKNOWN;
  memset(&cmd.color_spec, 0, sizeof(cmd.color_spec));
  cmd.config = *AppConfig_Get();
  cmd.config_mask = 0UL;
  cmd.raw_line = line;

  if (strcmp(line, "[$PING]") == 0)
  {
    cmd.type = COMM_CMD_PING;
  }
  else if (strcmp(line, "[$RESET]") == 0)
  {
    cmd.type = COMM_CMD_RESET;
  }
  else if (strcmp(line, "[$HARD_RESET]") == 0)
  {
    cmd.type = COMM_CMD_HARD_RESET;
  }
  else if (strcmp(line, "[$ColorDebug]") == 0)
  {
    cmd.type = COMM_CMD_COLOR_DEBUG;
  }
  else if (strcmp(line, "[$StopColor]") == 0)
  {
    cmd.type = COMM_CMD_STOP_COLOR;
  }
  else if (strcmp(line, "[$GET_CFG]") == 0)
  {
    cmd.type = COMM_CMD_GET_CONFIG;
  }
  else if (Comm_ParseConfigLine(line, &cmd) != 0U)
  {
    cmd.raw_line = line;
  }
  else if (Comm_ParseHomeColorLine(line, &cmd) != 0U)
  {
    cmd.raw_line = line;
  }
  else if (Comm_ParseTargetLine(line, &cmd) != 0U)
  {
    cmd.raw_line = line;
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

  DEBUG_PRINT("UART error code=%lu", (unsigned long)huart->ErrorCode);
  Comm_RecoverUart(huart);
}

