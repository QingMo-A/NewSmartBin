#include "gate.h"

#define GATE_PLACEHOLDER_MOVE_MS 500U

typedef enum
{
  GATE_STATE_IDLE = 0,
  GATE_STATE_OPENING,
  GATE_STATE_CLOSING
} Gate_State_t;

static struct
{
  Gate_State_t state;
  uint8_t is_open;
  uint32_t action_start_tick;
} s_gate;

void Gate_Init(void)
{
  s_gate.state = GATE_STATE_IDLE;
  s_gate.is_open = 0U;
  s_gate.action_start_tick = 0U;
}

void Gate_Update(void)
{
  if (s_gate.state == GATE_STATE_IDLE)
  {
    return;
  }

  if ((HAL_GetTick() - s_gate.action_start_tick) < GATE_PLACEHOLDER_MOVE_MS)
  {
    return;
  }

  if (s_gate.state == GATE_STATE_OPENING)
  {
    s_gate.is_open = 1U;
  }
  else if (s_gate.state == GATE_STATE_CLOSING)
  {
    s_gate.is_open = 0U;
  }

  s_gate.state = GATE_STATE_IDLE;
}

void Gate_Open(void)
{
  s_gate.state = GATE_STATE_OPENING;
  s_gate.action_start_tick = HAL_GetTick();
}

void Gate_Close(void)
{
  s_gate.state = GATE_STATE_CLOSING;
  s_gate.action_start_tick = HAL_GetTick();
}

uint8_t Gate_IsOpenDone(void)
{
  return (uint8_t)((s_gate.state == GATE_STATE_IDLE) && (s_gate.is_open != 0U));
}

uint8_t Gate_IsCloseDone(void)
{
  return (uint8_t)((s_gate.state == GATE_STATE_IDLE) && (s_gate.is_open == 0U));
}
