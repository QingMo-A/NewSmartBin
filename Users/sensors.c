#include "sensors.h"

#include "TCS34725.h"
#include "debug_io.h"

#include <string.h>

typedef struct
{
  uint8_t tcs_ready;
  uint8_t target_bin;
  Comm_ColorSpec_t target_color_spec;
  uint8_t bin_confirmed;
  uint8_t home_confirmed;
  uint8_t last_r;
  uint8_t last_g;
  uint8_t last_b;
  uint32_t last_update_tick;
  uint32_t last_debug_tick;
} Sensors_Context_t;

static Sensors_Context_t s_sensors;

static uint8_t Sensors_ChannelDiff(uint8_t a, uint8_t b)
{
  return (uint8_t)((a >= b) ? (a - b) : (b - a));
}

static uint8_t Sensors_IsWhiteHomeMatch(RGB raw_rgb, uint8_t r, uint8_t g, uint8_t b)
{
  if (raw_rgb.C < 1200U)
  {
    return 0U;
  }

  if ((r < 170U) || (g < 170U) || (b < 170U))
  {
    return 0U;
  }

  if (Sensors_ChannelDiff(r, g) > 35U)
  {
    return 0U;
  }
  if (Sensors_ChannelDiff(r, b) > 35U)
  {
    return 0U;
  }
  if (Sensors_ChannelDiff(g, b) > 35U)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t Sensors_IsChannelMatch(uint8_t value, uint8_t target, uint8_t tolerance)
{
  uint16_t lower;
  uint16_t upper;

  lower = (target > tolerance) ? (uint16_t)(target - tolerance) : 0U;
  upper = (uint16_t)target + tolerance;
  if (upper > 255U)
  {
    upper = 255U;
  }

  return (uint8_t)(((uint16_t)value >= lower) && ((uint16_t)value <= upper));
}

static uint8_t Sensors_IsColorMatch(uint8_t r, uint8_t g, uint8_t b, const Comm_ColorSpec_t *spec)
{
  if (spec == NULL)
  {
    return 0U;
  }

  return (uint8_t)(Sensors_IsChannelMatch(r, spec->r, spec->tol_r) != 0U &&
                   Sensors_IsChannelMatch(g, spec->g, spec->tol_g) != 0U &&
                   Sensors_IsChannelMatch(b, spec->b, spec->tol_b) != 0U);
}

void Sensors_Init(void)
{
  s_sensors.tcs_ready = (uint8_t)((TCS34725_Init() == 0U) ? 1U : 0U);
  s_sensors.target_bin = 0U;
  memset(&s_sensors.target_color_spec, 0, sizeof(s_sensors.target_color_spec));
  s_sensors.bin_confirmed = 0U;
  s_sensors.home_confirmed = 0U;
  s_sensors.last_r = 0U;
  s_sensors.last_g = 0U;
  s_sensors.last_b = 0U;
  s_sensors.last_update_tick = 0U;
  s_sensors.last_debug_tick = 0U;

  DEBUG_PRINT("SENS init %s", (s_sensors.tcs_ready != 0U) ? "ok" : "fail");
}

void Sensors_Update(void)
{
  RGB raw_rgb;
  uint32_t rgb888;

  if (s_sensors.tcs_ready == 0U)
  {
    s_sensors.bin_confirmed = 0U;
    s_sensors.home_confirmed = 0U;
    return;
  }

  if ((HAL_GetTick() - s_sensors.last_update_tick) < 80U)
  {
    return;
  }

  s_sensors.last_update_tick = HAL_GetTick();
  raw_rgb = TCS34725_Get_RGBData();
  rgb888 = TCS34725_GetRGB888(raw_rgb);

  s_sensors.last_r = (uint8_t)(rgb888 >> 16);
  s_sensors.last_g = (uint8_t)(rgb888 >> 8);
  s_sensors.last_b = (uint8_t)rgb888;

  s_sensors.home_confirmed = Sensors_IsWhiteHomeMatch(raw_rgb, s_sensors.last_r, s_sensors.last_g, s_sensors.last_b);

  if (s_sensors.target_bin != 0U)
  {
    s_sensors.bin_confirmed = Sensors_IsColorMatch(s_sensors.last_r,
                                                   s_sensors.last_g,
                                                   s_sensors.last_b,
                                                   &s_sensors.target_color_spec);
  }
  else
  {
    s_sensors.bin_confirmed = 0U;
  }

  if ((HAL_GetTick() - s_sensors.last_debug_tick) >= 250U)
  {
    s_sensors.last_debug_tick = HAL_GetTick();
    DEBUG_PRINT("SENS C=%u RGB=%u,%u,%u tgt=%u rgb=%u,%u,%u tol=%u,%u,%u bin=%u home=%u",
                raw_rgb.C,
                s_sensors.last_r,
                s_sensors.last_g,
                s_sensors.last_b,
                s_sensors.target_bin,
                s_sensors.target_color_spec.r,
                s_sensors.target_color_spec.g,
                s_sensors.target_color_spec.b,
                s_sensors.target_color_spec.tol_r,
                s_sensors.target_color_spec.tol_g,
                s_sensors.target_color_spec.tol_b,
                s_sensors.bin_confirmed,
                s_sensors.home_confirmed);
  }
}

void Sensors_SetTarget(uint8_t bin_id, const Comm_ColorSpec_t *color_spec)
{
  s_sensors.target_bin = bin_id;
  if (color_spec != NULL)
  {
    s_sensors.target_color_spec = *color_spec;
  }
  else
  {
    memset(&s_sensors.target_color_spec, 0, sizeof(s_sensors.target_color_spec));
  }
  s_sensors.bin_confirmed = 0U;
}

void Sensors_ClearTarget(void)
{
  s_sensors.target_bin = 0U;
  memset(&s_sensors.target_color_spec, 0, sizeof(s_sensors.target_color_spec));
  s_sensors.bin_confirmed = 0U;
}

uint8_t Sensors_IsBinConfirmed(uint8_t bin_id)
{
  if ((bin_id == 0U) || (bin_id != s_sensors.target_bin))
  {
    return 0U;
  }

  return s_sensors.bin_confirmed;
}

uint8_t Sensors_IsHomeConfirmed(void)
{
  return s_sensors.home_confirmed;
}
