#include "app.h"

#include "app_config.h"
#include "debug_io.h"
#include "gate.h"
#include "motion.h"
#include "sensors.h"

#include <string.h>

typedef struct
{
  App_State_t state;
  uint8_t active_bin;
  Comm_Direction_t active_direction;
  Comm_ColorSpec_t active_color_spec;
  uint8_t send_done_when_home;
  uint8_t drop_close_requested;
  uint32_t state_enter_tick;
  uint32_t drop_wait_tick;
  uint32_t error_blink_tick;
} App_Context_t;

static App_Context_t s_app;

static const char *App_StateName(App_State_t state)
{
  switch (state)
  {
    case APP_STATE_IDLE:
      return "IDLE";
    case APP_STATE_MOVING_TO_BIN:
      return "MOVING_TO_BIN";
    case APP_STATE_PREOPEN_WAIT:
      return "PREOPEN_WAIT";
    case APP_STATE_OPENING_GATE:
      return "OPENING_GATE";
    case APP_STATE_WAITING_DROP:
      return "WAITING_DROP";
    case APP_STATE_POSTCLOSE_WAIT:
      return "POSTCLOSE_WAIT";
    case APP_STATE_RETURNING_HOME:
      return "RETURNING_HOME";
    case APP_STATE_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

static const char *App_DirectionName(Comm_Direction_t direction)
{
  switch (direction)
  {
    case COMM_DIR_LEFT:
      return "LEFT";
    case COMM_DIR_RIGHT:
      return "RIGHT";
    default:
      return "UNKNOWN";
  }
}

static void App_SetState(App_State_t new_state)
{
  App_State_t old_state = s_app.state;

  s_app.state = new_state;
  s_app.state_enter_tick = HAL_GetTick();

  if (old_state != new_state)
  {
    DEBUG_PRINT("STATE=%s->%s", App_StateName(old_state), App_StateName(new_state));
  }
}

static uint32_t App_StateElapsed(void)
{
  uint32_t now = HAL_GetTick();

  if (now < s_app.state_enter_tick)
  {
    DEBUG_PRINT("WARN tick_backwards now=%lu enter=%lu state=%s",
                (unsigned long)now,
                (unsigned long)s_app.state_enter_tick,
                App_StateName(s_app.state));
    s_app.state_enter_tick = now;
    return 0U;
  }

  return now - s_app.state_enter_tick;
}

static uint8_t App_StateTimedOut(uint32_t timeout_ms)
{
  return (uint8_t)(App_StateElapsed() >= timeout_ms);
}

static void App_CompleteTask(void)
{
  Motion_Stop();
  s_app.active_bin = 0U;
  s_app.active_direction = COMM_DIR_UNKNOWN;
  memset(&s_app.active_color_spec, 0, sizeof(s_app.active_color_spec));
  s_app.drop_close_requested = 0U;
  Sensors_ClearTarget();
  App_SetState(APP_STATE_IDLE);
  Debug_UserLedSet(0U);
  DEBUG_PRINT("TASK done_home");

  if (s_app.send_done_when_home != 0U)
  {
    s_app.send_done_when_home = 0U;
    (void)Comm_SendLine(COMM_TX_DONE_HOME);
  }
}

static void App_EnterError(void)
{
  App_State_t prev_state = s_app.state;

  Motion_Stop();
  s_app.active_bin = 0U;
  s_app.active_direction = COMM_DIR_UNKNOWN;
  memset(&s_app.active_color_spec, 0, sizeof(s_app.active_color_spec));
  s_app.send_done_when_home = 0U;
  s_app.drop_close_requested = 0U;
  Sensors_ClearTarget();
  Gate_Close();
  App_SetState(APP_STATE_ERROR);
  s_app.error_blink_tick = HAL_GetTick();
  DEBUG_PRINT("ERROR enter from=%s", App_StateName(prev_state));
  (void)Comm_SendLine(COMM_TX_ERR);
}

void App_Init(void)
{
  AppConfig_Init();
  Motion_Init();
  Gate_Init();
  Sensors_Init();

  s_app.active_bin = 0U;
  s_app.active_direction = COMM_DIR_UNKNOWN;
  memset(&s_app.active_color_spec, 0, sizeof(s_app.active_color_spec));
  s_app.send_done_when_home = 0U;
  s_app.drop_close_requested = 0U;
  s_app.drop_wait_tick = 0U;
  s_app.error_blink_tick = 0U;
  App_SetState(APP_STATE_IDLE);
  Gate_Close();
  Sensors_ClearTarget();
  Debug_UserLedSet(0U);
  DEBUG_PRINT("APP init");
}

HAL_StatusTypeDef App_StartTask(uint8_t bin_id, Comm_Direction_t direction, const Comm_ColorSpec_t *color_spec)
{
  if ((bin_id < 1U) || (bin_id > 4U) || (color_spec == NULL))
  {
    return HAL_ERROR;
  }

  if (direction == COMM_DIR_UNKNOWN)
  {
    return HAL_ERROR;
  }

  if (s_app.state != APP_STATE_IDLE)
  {
    return HAL_BUSY;
  }

  s_app.active_bin = bin_id;
  s_app.active_direction = direction;
  s_app.active_color_spec = *color_spec;
  s_app.send_done_when_home = 1U;
  s_app.drop_close_requested = 0U;
  Sensors_SetTarget(bin_id, color_spec);
  DEBUG_PRINT("TASK start bin=%u dir=%s rgb=%u,%u,%u tol=%u,%u,%u",
              bin_id,
              App_DirectionName(direction),
              color_spec->r,
              color_spec->g,
              color_spec->b,
              color_spec->tol_r,
              color_spec->tol_g,
              color_spec->tol_b);
  Motion_MoveToBin(bin_id, direction);
  App_SetState(APP_STATE_MOVING_TO_BIN);
  Debug_UserLedSet(1U);
  return HAL_OK;
}

void App_RequestReset(void)
{
  DEBUG_PRINT("RESET request");
  s_app.active_bin = 0U;
  s_app.active_direction = COMM_DIR_UNKNOWN;
  memset(&s_app.active_color_spec, 0, sizeof(s_app.active_color_spec));
  s_app.send_done_when_home = 0U;
  s_app.drop_close_requested = 1U;
  Sensors_ClearTarget();
  Gate_Close();

  if (Sensors_IsHomeConfirmed() != 0U)
  {
    Motion_Stop();
    App_SetState(APP_STATE_IDLE);
    Debug_UserLedSet(0U);
  }
  else
  {
    Motion_ReturnHome();
    DEBUG_PRINT("MOVE return_home_cmd");
    App_SetState(APP_STATE_RETURNING_HOME);
    Debug_UserLedSet(1U);
  }
}

void App_RequestHardReset(void)
{
  DEBUG_PRINT("HARD_RESET request");
  Motion_Stop();
  s_app.active_bin = 0U;
  s_app.active_direction = COMM_DIR_UNKNOWN;
  memset(&s_app.active_color_spec, 0, sizeof(s_app.active_color_spec));
  s_app.send_done_when_home = 0U;
  s_app.drop_close_requested = 0U;
  s_app.drop_wait_tick = 0U;
  s_app.error_blink_tick = 0U;
  Sensors_ClearTarget();
  Gate_Close();
  App_SetState(APP_STATE_IDLE);
  Debug_UserLedSet(0U);
}

void App_OnCommandReceived(const Comm_Command_t *cmd)
{
  if (cmd == NULL)
  {
    DEBUG_PRINT("CMD null");
    (void)Comm_SendLine(COMM_TX_ERR);
    return;
  }

  switch (cmd->type)
  {
    case COMM_CMD_PING:
      DEBUG_PRINT("CMD ping");
      (void)Comm_SendLine(COMM_TX_ACK);
      break;

    case COMM_CMD_RESET:
      DEBUG_PRINT("CMD reset");
      (void)Comm_SendLine(COMM_TX_ACK);
      App_RequestReset();
      break;

    case COMM_CMD_HARD_RESET:
      DEBUG_PRINT("CMD hard_reset");
      (void)Comm_SendLine(COMM_TX_ACK);
      App_RequestHardReset();
      break;

    case COMM_CMD_TARGET_BIN:
      if (s_app.state == APP_STATE_IDLE)
      {
        DEBUG_PRINT("CMD target bin=%u dir=%s rgb=%u,%u,%u tol=%u,%u,%u",
                    cmd->bin_id,
                    App_DirectionName(cmd->direction),
                    cmd->color_spec.r,
                    cmd->color_spec.g,
                    cmd->color_spec.b,
                    cmd->color_spec.tol_r,
                    cmd->color_spec.tol_g,
                    cmd->color_spec.tol_b);
        (void)Comm_SendLine(COMM_TX_ACK);
        if (App_StartTask(cmd->bin_id, cmd->direction, &cmd->color_spec) != HAL_OK)
        {
          App_EnterError();
        }
      }
      else if (s_app.state == APP_STATE_ERROR)
      {
        DEBUG_PRINT("CMD target_rejected error_state");
        (void)Comm_SendLine(COMM_TX_ERR);
      }
      else
      {
        DEBUG_PRINT("CMD target_busy state=%s", App_StateName(s_app.state));
        (void)Comm_SendLine(COMM_TX_BUSY);
      }
      break;

    case COMM_CMD_COLOR_DEBUG:
      Sensors_SetColorDebugEnabled(1U);
      DEBUG_PRINT("CMD color_debug on");
      (void)Comm_SendLine(COMM_TX_ACK);
      break;

    case COMM_CMD_STOP_COLOR:
      Sensors_SetColorDebugEnabled(0U);
      DEBUG_PRINT("CMD color_debug off");
      (void)Comm_SendLine(COMM_TX_ACK);
      break;

    case COMM_CMD_CONFIG:
      if (AppConfig_Apply(&cmd->config, cmd->config_mask) == HAL_OK)
      {
        DEBUG_PRINT("CMD cfg applied mask=0x%lX", (unsigned long)cmd->config_mask);
        (void)Comm_SendLine(COMM_TX_ACK);
      }
      else
      {
        DEBUG_PRINT("CMD cfg invalid mask=0x%lX", (unsigned long)cmd->config_mask);
        (void)Comm_SendLine(COMM_TX_ERR);
      }
      break;

    case COMM_CMD_GET_CONFIG:
    {
      char cfg_line[160];

      if (AppConfig_Format(cfg_line, sizeof(cfg_line)) == HAL_OK)
      {
        (void)Comm_SendLine(cfg_line);
      }
      else
      {
        (void)Comm_SendLine(COMM_TX_ERR);
      }
      break;
    }

    case COMM_CMD_HOME_COLOR:
      Sensors_SetHomeColor(&cmd->color_spec);
      DEBUG_PRINT("CMD home rgb=%u,%u,%u tol=%u,%u,%u",
                  cmd->color_spec.r,
                  cmd->color_spec.g,
                  cmd->color_spec.b,
                  cmd->color_spec.tol_r,
                  cmd->color_spec.tol_g,
                  cmd->color_spec.tol_b);
      (void)Comm_SendLine(COMM_TX_ACK);
      break;

    case COMM_CMD_INVALID:
    case COMM_CMD_NONE:
    default:
      DEBUG_PRINT("CMD invalid");
      (void)Comm_SendLine(COMM_TX_ERR);
      break;
  }
}

void App_Process(void)
{
  const App_Config_t *config = AppConfig_Get();

  Sensors_Update();
  Motion_Update();
  Gate_Update();

  switch (s_app.state)
  {
    case APP_STATE_IDLE:
      Debug_UserLedSet(0U);
      break;

    case APP_STATE_MOVING_TO_BIN:
      if (Sensors_IsBinConfirmed(s_app.active_bin) != 0U)
      {
        Motion_Stop();
        DEBUG_PRINT("MOVE reached bin=%u", s_app.active_bin);
        App_SetState(APP_STATE_PREOPEN_WAIT);
      }
      else if (App_StateTimedOut(config->move_timeout_ms) != 0U)
      {
        DEBUG_PRINT("TIMEOUT move_to_bin elapsed=%lu limit=%lu",
                    (unsigned long)App_StateElapsed(),
                    (unsigned long)config->move_timeout_ms);
        App_EnterError();
      }
      break;

    case APP_STATE_PREOPEN_WAIT:
      if (App_StateTimedOut(config->preopen_delay_ms) != 0U)
      {
        DEBUG_PRINT("GATE preopen_wait_done");
        Gate_Open();
        DEBUG_PRINT("GATE open_cmd");
        App_SetState(APP_STATE_OPENING_GATE);
      }
      break;

    case APP_STATE_OPENING_GATE:
      if (Gate_IsOpenDone() != 0U)
      {
        DEBUG_PRINT("GATE opened");
        s_app.drop_close_requested = 0U;
        s_app.drop_wait_tick = HAL_GetTick();
        App_SetState(APP_STATE_WAITING_DROP);
      }
      else if (App_StateTimedOut(config->gate_timeout_ms) != 0U)
      {
        DEBUG_PRINT("TIMEOUT gate_open elapsed=%lu limit=%lu",
                    (unsigned long)App_StateElapsed(),
                    (unsigned long)config->gate_timeout_ms);
        App_EnterError();
      }
      break;

    case APP_STATE_WAITING_DROP:
      if (s_app.drop_close_requested == 0U)
      {
        if ((HAL_GetTick() - s_app.drop_wait_tick) >= config->drop_wait_ms)
        {
          DEBUG_PRINT("DROP wait_done");
          Gate_Close();
          DEBUG_PRINT("GATE close_cmd");
          s_app.drop_close_requested = 1U;
          s_app.state_enter_tick = HAL_GetTick();
        }
      }
      else if (Gate_IsCloseDone() != 0U)
      {
        DEBUG_PRINT("GATE closed");
        App_SetState(APP_STATE_POSTCLOSE_WAIT);
      }
      else if (App_StateTimedOut(config->gate_timeout_ms) != 0U)
      {
        DEBUG_PRINT("TIMEOUT gate_close elapsed=%lu limit=%lu",
                    (unsigned long)App_StateElapsed(),
                    (unsigned long)config->gate_timeout_ms);
        App_EnterError();
      }
      break;

    case APP_STATE_POSTCLOSE_WAIT:
      if (App_StateTimedOut(config->postclose_delay_ms) != 0U)
      {
        DEBUG_PRINT("MOVE postclose_wait_done");
        Motion_ReturnHome();
        DEBUG_PRINT("MOVE return_home_cmd");
        App_SetState(APP_STATE_RETURNING_HOME);
      }
      break;

    case APP_STATE_RETURNING_HOME:
      if (Sensors_IsHomeConfirmed() != 0U)
      {
        Motion_Stop();
        DEBUG_PRINT("HOME reached");
        App_CompleteTask();
      }
      else if (App_StateTimedOut(config->return_home_timeout_ms) != 0U)
      {
        DEBUG_PRINT("TIMEOUT return_home elapsed=%lu limit=%lu",
                    (unsigned long)App_StateElapsed(),
                    (unsigned long)config->return_home_timeout_ms);
        App_EnterError();
      }
      break;

    case APP_STATE_ERROR:
      if ((HAL_GetTick() - s_app.error_blink_tick) >= config->error_blink_ms)
      {
        s_app.error_blink_tick = HAL_GetTick();
        Debug_UserLedToggle();
      }
      break;

    default:
      DEBUG_PRINT("ERROR invalid_state");
      App_EnterError();
      break;
  }
}

App_State_t App_GetState(void)
{
  return s_app.state;
}

uint8_t App_IsBusy(void)
{
  return (uint8_t)(s_app.state != APP_STATE_IDLE);
}



