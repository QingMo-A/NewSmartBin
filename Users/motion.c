#include "motion.h"

#define MOTION_PLACEHOLDER_TRAVEL_MS 1200U

typedef enum
{
  MOTION_STATE_IDLE = 0,
  MOTION_STATE_TO_BIN,
  MOTION_STATE_TO_HOME
} Motion_State_t;

static struct
{
  Motion_State_t state;
  uint8_t current_bin;
  uint8_t target_bin;
  uint8_t at_home;
  uint32_t action_start_tick;
} s_motion;

void Motion_Init(void)
{
  s_motion.state = MOTION_STATE_IDLE;
  s_motion.current_bin = 0U;
  s_motion.target_bin = 0U;
  s_motion.at_home = 1U;
  s_motion.action_start_tick = 0U;
}

void Motion_Update(void)
{
  if (s_motion.state == MOTION_STATE_IDLE)
  {
    return;
  }

  if ((HAL_GetTick() - s_motion.action_start_tick) < MOTION_PLACEHOLDER_TRAVEL_MS)
  {
    return;
  }

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

  s_motion.state = MOTION_STATE_IDLE;
}

void Motion_MoveToBin(uint8_t bin_id)
{
  s_motion.target_bin = bin_id;
  s_motion.state = MOTION_STATE_TO_BIN;
  s_motion.action_start_tick = HAL_GetTick();
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
  s_motion.action_start_tick = HAL_GetTick();
}

uint8_t Motion_IsAtHome(void)
{
  return (uint8_t)((s_motion.state == MOTION_STATE_IDLE) && (s_motion.at_home != 0U));
}
