#include "motion.h"

#include "app_config.h"
#include "Motor_L9110S.h"

typedef enum
{
  MOTION_STATE_IDLE = 0,
  MOTION_STATE_TO_BIN,
  MOTION_STATE_TO_HOME,
  MOTION_STATE_MANUAL
} Motion_State_t;

typedef struct
{
  Motion_State_t state;
  uint8_t current_bin;
  uint8_t target_bin;
  uint8_t at_home;
  Comm_Direction_t target_direction;
  Comm_Direction_t last_move_direction;
  Comm_Direction_t manual_direction;
  uint32_t manual_start_tick;
  uint32_t manual_duration_ms;
} Motion_Context_t;

static Motion_Context_t s_motion;

static void Motion_RunDirection(Comm_Direction_t direction, int speed)
{
  if (direction == COMM_DIR_LEFT)
  {
    Motor_Run(-speed, -speed);
  }
  else if (direction == COMM_DIR_RIGHT)
  {
    Motor_Run(speed, speed);
  }
  else
  {
    Motor_Stop();
  }
}

void Motion_Init(void)
{
  s_motion.state = MOTION_STATE_IDLE;
  s_motion.current_bin = 0U;
  s_motion.target_bin = 0U;
  s_motion.at_home = 1U;
  s_motion.target_direction = COMM_DIR_UNKNOWN;
  s_motion.last_move_direction = COMM_DIR_UNKNOWN;
  s_motion.manual_direction = COMM_DIR_UNKNOWN;
  s_motion.manual_start_tick = 0U;
  s_motion.manual_duration_ms = 0U;
  Motor_Stop();
}

void Motion_Update(void)
{
  int speed = (int)AppConfig_Get()->motion_speed;

  switch (s_motion.state)
  {
    case MOTION_STATE_TO_BIN:
      Motion_RunDirection(s_motion.target_direction, speed);
      break;

    case MOTION_STATE_TO_HOME:
      if (s_motion.last_move_direction == COMM_DIR_LEFT)
      {
        Motion_RunDirection(COMM_DIR_RIGHT, speed);
      }
      else if (s_motion.last_move_direction == COMM_DIR_RIGHT)
      {
        Motion_RunDirection(COMM_DIR_LEFT, speed);
      }
      else
      {
        Motor_Stop();
      }
      break;

    case MOTION_STATE_MANUAL:
      if ((HAL_GetTick() - s_motion.manual_start_tick) >= s_motion.manual_duration_ms)
      {
        Motor_Stop();
        s_motion.state = MOTION_STATE_IDLE;
        s_motion.manual_direction = COMM_DIR_UNKNOWN;
        s_motion.manual_duration_ms = 0U;
      }
      else
      {
        Motion_RunDirection(s_motion.manual_direction, speed);
      }
      break;

    case MOTION_STATE_IDLE:
    default:
      break;
  }
}

void Motion_MoveToBin(uint8_t bin_id, Comm_Direction_t direction)
{
  s_motion.target_bin = bin_id;
  s_motion.target_direction = direction;
  s_motion.last_move_direction = direction;
  s_motion.state = MOTION_STATE_TO_BIN;
  s_motion.at_home = 0U;
}

void Motion_ManualRun(Comm_Direction_t direction, uint32_t duration_ms)
{
  if ((direction == COMM_DIR_UNKNOWN) || (duration_ms == 0U))
  {
    Motion_Stop();
    return;
  }

  s_motion.target_bin = 0U;
  s_motion.current_bin = 0U;
  s_motion.at_home = 0U;
  s_motion.target_direction = COMM_DIR_UNKNOWN;
  s_motion.manual_direction = direction;
  s_motion.manual_start_tick = HAL_GetTick();
  s_motion.manual_duration_ms = duration_ms;
  s_motion.state = MOTION_STATE_MANUAL;
}

uint8_t Motion_IsAtTarget(void)
{
  return (uint8_t)((s_motion.state == MOTION_STATE_IDLE) &&
                   (s_motion.at_home == 0U) &&
                   (s_motion.current_bin == s_motion.target_bin) &&
                   (s_motion.target_bin != 0U));
}

void Motion_ReturnHome(void)
{
  s_motion.state = MOTION_STATE_TO_HOME;
  s_motion.target_direction = COMM_DIR_UNKNOWN;
}

uint8_t Motion_IsAtHome(void)
{
  return (uint8_t)((s_motion.state == MOTION_STATE_IDLE) && (s_motion.at_home != 0U));
}

uint8_t Motion_IsBusy(void)
{
  return (uint8_t)(s_motion.state != MOTION_STATE_IDLE);
}

void Motion_Stop(void)
{
  if (s_motion.state == MOTION_STATE_TO_BIN)
  {
    s_motion.current_bin = s_motion.target_bin;
    s_motion.at_home = 0U;
  }
  else if (s_motion.state == MOTION_STATE_TO_HOME)
  {
    s_motion.current_bin = 0U;
    s_motion.target_bin = 0U;
    s_motion.at_home = 1U;
  }
  else if (s_motion.state == MOTION_STATE_MANUAL)
  {
    s_motion.current_bin = 0U;
    s_motion.target_bin = 0U;
    s_motion.at_home = 0U;
  }

  Motor_Stop();
  s_motion.state = MOTION_STATE_IDLE;
  s_motion.manual_direction = COMM_DIR_UNKNOWN;
  s_motion.manual_duration_ms = 0U;
}

