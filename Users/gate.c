#include "gate.h"

#include "Motor_Servo.h"

#define GATE_OPEN_ANGLE           0U
#define GATE_CLOSE_ANGLE          90U
#define GATE_OPEN_SETTLE_MS       500U
#define GATE_CLOSE_SETTLE_MS      500U

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
  Servo_Init();

  s_gate.state = GATE_STATE_IDLE;
  s_gate.is_open = 0U;
  s_gate.action_start_tick = HAL_GetTick();
}

void Gate_Update(void)
{
  uint32_t elapsed = HAL_GetTick() - s_gate.action_start_tick;

  switch (s_gate.state)
  {
    case GATE_STATE_IDLE:
      return;

    case GATE_STATE_OPENING:
      if (elapsed >= GATE_OPEN_SETTLE_MS)
      {
        s_gate.is_open = 1U;
        s_gate.state = GATE_STATE_IDLE;
      }
      return;

    case GATE_STATE_CLOSING:
      if (elapsed >= GATE_CLOSE_SETTLE_MS)
      {
        s_gate.is_open = 0U;
        s_gate.state = GATE_STATE_IDLE;
      }
      return;

    default:
      s_gate.state = GATE_STATE_IDLE;
      s_gate.is_open = 0U;
      return;
  }
}

void Gate_Open(void)
{
  Servo_SetDoorsSameAngle(GATE_OPEN_ANGLE);
  s_gate.state = GATE_STATE_OPENING;
  s_gate.action_start_tick = HAL_GetTick();
}

void Gate_Close(void)
{
  Servo_SetDoorsSameAngle(GATE_CLOSE_ANGLE);
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
