#include "HC-SR04.h"
#include "tim.h"

#define HCSR04_TRIGGER_PULSE_US     12U
#define HCSR04_WAIT_TIMEOUT_US      30000U
#define HCSR04_SOUND_SPEED_DIVISOR  58.0f

float g_hcsr04_distance_cm = -1.0f;
static uint32_t g_hcsr04_pulse_width_us = 0U;
static bool g_hcsr04_data_valid = false;

static uint32_t HCSR04_GetTickUs(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}

static uint32_t HCSR04_GetElapsedUs(uint32_t start, uint32_t now)
{
  const uint32_t reload = __HAL_TIM_GET_AUTORELOAD(&htim2);

  if (now >= start)
  {
    return now - start;
  }

  return (reload - start + 1U) + now;
}

static void HCSR04_DelayUs(uint32_t us)
{
  const uint32_t start = HCSR04_GetTickUs();

  while (HCSR04_GetElapsedUs(start, HCSR04_GetTickUs()) < us)
  {
  }
}

void HCSR04_Init(void)
{
  HAL_TIM_Base_Start(&htim2);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);

  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
  g_hcsr04_distance_cm = -1.0f;
  g_hcsr04_pulse_width_us = 0U;
  g_hcsr04_data_valid = false;
}

bool HCSR04_Measure(void)
{
  uint32_t start_tick;
  uint32_t end_tick;

  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
  HCSR04_DelayUs(2U);
  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
  HCSR04_DelayUs(HCSR04_TRIGGER_PULSE_US);
  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

  start_tick = HCSR04_GetTickUs();
  while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_RESET)
  {
    if (HCSR04_GetElapsedUs(start_tick, HCSR04_GetTickUs()) > HCSR04_WAIT_TIMEOUT_US)
    {
      g_hcsr04_distance_cm = -1.0f;
      g_hcsr04_pulse_width_us = 0U;
      g_hcsr04_data_valid = false;
      return false;
    }
  }

  start_tick = HCSR04_GetTickUs();
  while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET)
  {
    if (HCSR04_GetElapsedUs(start_tick, HCSR04_GetTickUs()) > HCSR04_WAIT_TIMEOUT_US)
    {
      g_hcsr04_distance_cm = -1.0f;
      g_hcsr04_pulse_width_us = 0U;
      g_hcsr04_data_valid = false;
      return false;
    }
  }

  end_tick = HCSR04_GetTickUs();
  g_hcsr04_pulse_width_us = HCSR04_GetElapsedUs(start_tick, end_tick);
  g_hcsr04_distance_cm = g_hcsr04_pulse_width_us / HCSR04_SOUND_SPEED_DIVISOR;
  g_hcsr04_data_valid = true;
  return true;
}

float HCSR04_GetDistanceCm(void)
{
  return g_hcsr04_distance_cm;
}

uint32_t HCSR04_GetPulseWidthUs(void)
{
  return g_hcsr04_pulse_width_us;
}

bool HCSR04_IsDataValid(void)
{
  return g_hcsr04_data_valid;
}
