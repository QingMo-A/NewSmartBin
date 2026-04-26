#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "comm.h"

typedef enum
{
  APP_STATE_IDLE = 0,
  APP_STATE_MOVING_TO_BIN,
  APP_STATE_PREOPEN_WAIT,
  APP_STATE_OPENING_GATE,
  APP_STATE_WAITING_DROP,
  APP_STATE_POSTCLOSE_WAIT,
  APP_STATE_RETURNING_HOME,
  APP_STATE_MANUAL_CONTROL,
  APP_STATE_ERROR
} App_State_t;

void App_Init(void);
void App_Process(void);
HAL_StatusTypeDef App_StartTask(uint8_t bin_id, Comm_Direction_t direction, const Comm_ColorSpec_t *color_spec);
void App_OnCommandReceived(const Comm_Command_t *cmd);
void App_RequestReset(void);
void App_RequestHardReset(void);
App_State_t App_GetState(void);
uint8_t App_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
