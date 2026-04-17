#ifndef COMM_H
#define COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  COMM_CMD_NONE = 0,
  COMM_CMD_TARGET_BIN,
  COMM_CMD_PING,
  COMM_CMD_RESET,
  COMM_CMD_INVALID
} Comm_CommandType_t;

typedef enum
{
  COMM_DIR_UNKNOWN = 0,
  COMM_DIR_LEFT,
  COMM_DIR_RIGHT
} Comm_Direction_t;

typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t tol_r;
  uint8_t tol_g;
  uint8_t tol_b;
} Comm_ColorSpec_t;

typedef struct
{
  Comm_CommandType_t type;
  uint8_t bin_id;
  Comm_Direction_t direction;
  Comm_ColorSpec_t color_spec;
  const char *raw_line;
} Comm_Command_t;

typedef void (*Comm_CommandHandler_t)(const Comm_Command_t *cmd);

#define COMM_TX_ACK       "[$ACK]"
#define COMM_TX_BUSY      "[$BUSY]"
#define COMM_TX_ERR       "[$ERR]"
#define COMM_TX_DONE_HOME "[$DONE_HOME]"

void Comm_Init(UART_HandleTypeDef *huart, Comm_CommandHandler_t handler);
void Comm_Task(void);
void Comm_RxByteCallback(UART_HandleTypeDef *huart);
void Comm_ErrorCallback(UART_HandleTypeDef *huart);
void Comm_ProcessLine(const char *line);
HAL_StatusTypeDef Comm_SendLine(const char *line);
UART_HandleTypeDef *Comm_GetUartHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* COMM_H */
