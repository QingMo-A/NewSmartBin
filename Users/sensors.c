#include "sensors.h"

#include "TCS34725.h"
#include "debug_io.h"

#include <string.h>

#define SENSORS_UPDATE_PERIOD_MS       80U
#define SENSORS_DEBUG_PERIOD_MS        250U

#define SENSORS_BIN_ENTER_COUNT        3U
#define SENSORS_BIN_EXIT_COUNT         2U
#define SENSORS_HOME_ENTER_COUNT       4U
#define SENSORS_HOME_EXIT_COUNT        3U

#define SENSORS_RATIO_MIN_TOL          8U
#define SENSORS_HOME_DEFAULT_R         85U
#define SENSORS_HOME_DEFAULT_G         85U
#define SENSORS_HOME_DEFAULT_B         85U
#define SENSORS_HOME_DEFAULT_TOL_R     20U
#define SENSORS_HOME_DEFAULT_TOL_G     20U
#define SENSORS_HOME_DEFAULT_TOL_B     20U

typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} Sensors_RatioRgb_t;

typedef struct
{
  uint8_t tcs_ready;
  uint8_t target_bin;
  Comm_ColorSpec_t target_color_spec;
  Comm_ColorSpec_t home_color_spec;
  uint8_t bin_confirmed;
  uint8_t home_confirmed;
  uint8_t manual_color_debug_enabled;
  uint8_t auto_color_debug_enabled;
  uint8_t bin_enter_count;
  uint8_t bin_exit_count;
  uint8_t home_enter_count;
  uint8_t home_exit_count;
  uint8_t last_r;
  uint8_t last_g;
  uint8_t last_b;
  Sensors_RatioRgb_t last_ratio;
  uint32_t last_update_tick;
  uint32_t last_debug_tick;
} Sensors_Context_t;

static Sensors_Context_t s_sensors;

static uint8_t Sensors_IsAnyColorDebugEnabled(void)
{
  return (uint8_t)((s_sensors.manual_color_debug_enabled != 0U) ||
                   (s_sensors.auto_color_debug_enabled != 0U));
}

static uint8_t Sensors_ChannelDiff(uint8_t a, uint8_t b)
{
  return (uint8_t)((a >= b) ? (a - b) : (b - a));
}

static uint8_t Sensors_ChannelMax(uint8_t a, uint8_t b, uint8_t c)
{
  uint8_t max_value = a;

  if (b > max_value)
  {
    max_value = b;
  }
  if (c > max_value)
  {
    max_value = c;
  }

  return max_value;
}

static uint8_t Sensors_ChannelMin(uint8_t a, uint8_t b, uint8_t c)
{
  uint8_t min_value = a;

  if (b < min_value)
  {
    min_value = b;
  }
  if (c < min_value)
  {
    min_value = c;
  }

  return min_value;
}

static Sensors_RatioRgb_t Sensors_MakeRatioRgb(uint16_t r, uint16_t g, uint16_t b)
{
  Sensors_RatioRgb_t ratio = {0U, 0U, 0U};
  uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;

  if (sum == 0U)
  {
    return ratio;
  }

  ratio.r = (uint8_t)(((uint32_t)r * 255U) / sum);
  ratio.g = (uint8_t)(((uint32_t)g * 255U) / sum);
  ratio.b = (uint8_t)(((uint32_t)b * 255U) / sum);
  return ratio;
}

static uint8_t Sensors_DominantMask(uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t max_value = Sensors_ChannelMax(r, g, b);
  uint8_t mask = 0U;

  if (Sensors_ChannelDiff(r, max_value) <= 25U)
  {
    mask |= 0x01U;
  }
  if (Sensors_ChannelDiff(g, max_value) <= 25U)
  {
    mask |= 0x02U;
  }
  if (Sensors_ChannelDiff(b, max_value) <= 25U)
  {
    mask |= 0x04U;
  }

  return mask;
}

static uint8_t Sensors_ScaleToleranceToRatio(uint8_t tolerance, uint16_t target_sum)
{
  uint16_t scaled;

  if (target_sum == 0U)
  {
    return 0U;
  }

  scaled = (uint16_t)(((uint32_t)tolerance * 255U) / target_sum);
  if (scaled < SENSORS_RATIO_MIN_TOL)
  {
    scaled = SENSORS_RATIO_MIN_TOL;
  }
  if (scaled > 255U)
  {
    scaled = 255U;
  }

  return (uint8_t)scaled;
}

static uint8_t Sensors_IsRatioChannelMatch(uint8_t value, uint8_t target, uint8_t tolerance)
{
  return (uint8_t)(Sensors_ChannelDiff(value, target) <= tolerance);
}

static uint8_t Sensors_IsTargetColorMatch(RGB raw_rgb, const Comm_ColorSpec_t *spec, Sensors_RatioRgb_t *current_ratio_out)
{
  uint16_t target_sum;
  Sensors_RatioRgb_t current_ratio;
  Sensors_RatioRgb_t target_ratio;
  uint8_t tol_r;
  uint8_t tol_g;
  uint8_t tol_b;
  uint8_t target_mask;
  uint8_t current_mask;

  if (spec == NULL)
  {
    return 0U;
  }

  target_sum = (uint16_t)spec->r + (uint16_t)spec->g + (uint16_t)spec->b;
  if (target_sum == 0U)
  {
    return 0U;
  }

  current_ratio = Sensors_MakeRatioRgb(raw_rgb.R, raw_rgb.G, raw_rgb.B);
  target_ratio = Sensors_MakeRatioRgb(spec->r, spec->g, spec->b);

  if (current_ratio_out != NULL)
  {
    *current_ratio_out = current_ratio;
  }

  target_mask = Sensors_DominantMask(spec->r, spec->g, spec->b);
  current_mask = Sensors_DominantMask(current_ratio.r, current_ratio.g, current_ratio.b);
  if ((target_mask & current_mask) == 0U)
  {
    return 0U;
  }

  tol_r = Sensors_ScaleToleranceToRatio(spec->tol_r, target_sum);
  tol_g = Sensors_ScaleToleranceToRatio(spec->tol_g, target_sum);
  tol_b = Sensors_ScaleToleranceToRatio(spec->tol_b, target_sum);

  return (uint8_t)(Sensors_IsRatioChannelMatch(current_ratio.r, target_ratio.r, tol_r) != 0U &&
                   Sensors_IsRatioChannelMatch(current_ratio.g, target_ratio.g, tol_g) != 0U &&
                   Sensors_IsRatioChannelMatch(current_ratio.b, target_ratio.b, tol_b) != 0U);
}

static uint8_t Sensors_UpdateDebouncedFlag(uint8_t instant_match,
                                           uint8_t *confirmed,
                                           uint8_t *enter_count,
                                           uint8_t *exit_count,
                                           uint8_t enter_threshold,
                                           uint8_t exit_threshold)
{
  if ((confirmed == NULL) || (enter_count == NULL) || (exit_count == NULL))
  {
    return 0U;
  }

  if (instant_match != 0U)
  {
    if (*enter_count < enter_threshold)
    {
      ++(*enter_count);
    }
    *exit_count = 0U;

    if (*enter_count >= enter_threshold)
    {
      *confirmed = 1U;
    }
  }
  else
  {
    *enter_count = 0U;
    if (*confirmed != 0U)
    {
      if (*exit_count < exit_threshold)
      {
        ++(*exit_count);
      }
      if (*exit_count >= exit_threshold)
      {
        *confirmed = 0U;
      }
    }
    else
    {
      *exit_count = 0U;
    }
  }

  return *confirmed;
}

void Sensors_Init(void)
{
  s_sensors.tcs_ready = (uint8_t)((TCS34725_Init() == 0U) ? 1U : 0U);
  s_sensors.target_bin = 0U;
  memset(&s_sensors.target_color_spec, 0, sizeof(s_sensors.target_color_spec));
  s_sensors.home_color_spec.r = SENSORS_HOME_DEFAULT_R;
  s_sensors.home_color_spec.g = SENSORS_HOME_DEFAULT_G;
  s_sensors.home_color_spec.b = SENSORS_HOME_DEFAULT_B;
  s_sensors.home_color_spec.tol_r = SENSORS_HOME_DEFAULT_TOL_R;
  s_sensors.home_color_spec.tol_g = SENSORS_HOME_DEFAULT_TOL_G;
  s_sensors.home_color_spec.tol_b = SENSORS_HOME_DEFAULT_TOL_B;
  s_sensors.bin_confirmed = 0U;
  s_sensors.home_confirmed = 0U;
  s_sensors.manual_color_debug_enabled = 0U;
  s_sensors.auto_color_debug_enabled = 0U;
  s_sensors.bin_enter_count = 0U;
  s_sensors.bin_exit_count = 0U;
  s_sensors.home_enter_count = 0U;
  s_sensors.home_exit_count = 0U;
  s_sensors.last_r = 0U;
  s_sensors.last_g = 0U;
  s_sensors.last_b = 0U;
  memset(&s_sensors.last_ratio, 0, sizeof(s_sensors.last_ratio));
  s_sensors.last_update_tick = 0U;
  s_sensors.last_debug_tick = 0U;

  DEBUG_PRINT("SENS init %s", (s_sensors.tcs_ready != 0U) ? "ok" : "fail");
}

void Sensors_Update(void)
{
  RGB raw_rgb;
  RGB corrected_rgb;
  uint32_t rgb888;
  uint8_t instant_home_match;
  uint8_t instant_bin_match = 0U;

  if (s_sensors.tcs_ready == 0U)
  {
    s_sensors.bin_confirmed = 0U;
    s_sensors.home_confirmed = 0U;
    return;
  }

  if ((HAL_GetTick() - s_sensors.last_update_tick) < SENSORS_UPDATE_PERIOD_MS)
  {
    return;
  }

  s_sensors.last_update_tick = HAL_GetTick();
  raw_rgb = TCS34725_Get_RGBData();
  corrected_rgb = TCS34725_GetCalibratedRgb(raw_rgb);
  rgb888 = ((uint32_t)corrected_rgb.R << 16) | ((uint32_t)corrected_rgb.G << 8) | (uint32_t)corrected_rgb.B;

  s_sensors.last_r = (uint8_t)(rgb888 >> 16);
  s_sensors.last_g = (uint8_t)(rgb888 >> 8);
  s_sensors.last_b = (uint8_t)rgb888;

  instant_home_match = Sensors_IsTargetColorMatch(corrected_rgb, &s_sensors.home_color_spec, NULL);
  (void)Sensors_UpdateDebouncedFlag(instant_home_match,
                                    &s_sensors.home_confirmed,
                                    &s_sensors.home_enter_count,
                                    &s_sensors.home_exit_count,
                                    SENSORS_HOME_ENTER_COUNT,
                                    SENSORS_HOME_EXIT_COUNT);

  if (s_sensors.target_bin != 0U)
  {
    instant_bin_match = Sensors_IsTargetColorMatch(corrected_rgb,
                                                   &s_sensors.target_color_spec,
                                                   &s_sensors.last_ratio);
  }
  else
  {
    memset(&s_sensors.last_ratio, 0, sizeof(s_sensors.last_ratio));
  }

  (void)Sensors_UpdateDebouncedFlag(instant_bin_match,
                                    &s_sensors.bin_confirmed,
                                    &s_sensors.bin_enter_count,
                                    &s_sensors.bin_exit_count,
                                    SENSORS_BIN_ENTER_COUNT,
                                    SENSORS_BIN_EXIT_COUNT);

  if ((Sensors_IsAnyColorDebugEnabled() != 0U) && ((HAL_GetTick() - s_sensors.last_debug_tick) >= SENSORS_DEBUG_PERIOD_MS))
  {
    s_sensors.last_debug_tick = HAL_GetTick();
    DEBUG_PRINT("SENS C=%u RGB=%u,%u,%u ratio=%u,%u,%u tgt=%u rgb=%u,%u,%u tol=%u,%u,%u bin=%u home=%u",
                raw_rgb.C,
                s_sensors.last_r,
                s_sensors.last_g,
                s_sensors.last_b,
                s_sensors.last_ratio.r,
                s_sensors.last_ratio.g,
                s_sensors.last_ratio.b,
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
  s_sensors.bin_enter_count = 0U;
  s_sensors.bin_exit_count = 0U;
}

void Sensors_SetHomeColor(const Comm_ColorSpec_t *color_spec)
{
  if (color_spec != NULL)
  {
    s_sensors.home_color_spec = *color_spec;
  }
  else
  {
    s_sensors.home_color_spec.r = SENSORS_HOME_DEFAULT_R;
    s_sensors.home_color_spec.g = SENSORS_HOME_DEFAULT_G;
    s_sensors.home_color_spec.b = SENSORS_HOME_DEFAULT_B;
    s_sensors.home_color_spec.tol_r = SENSORS_HOME_DEFAULT_TOL_R;
    s_sensors.home_color_spec.tol_g = SENSORS_HOME_DEFAULT_TOL_G;
    s_sensors.home_color_spec.tol_b = SENSORS_HOME_DEFAULT_TOL_B;
  }
  s_sensors.home_confirmed = 0U;
  s_sensors.home_enter_count = 0U;
  s_sensors.home_exit_count = 0U;
}

void Sensors_ClearTarget(void)
{
  s_sensors.target_bin = 0U;
  memset(&s_sensors.target_color_spec, 0, sizeof(s_sensors.target_color_spec));
  s_sensors.bin_confirmed = 0U;
  s_sensors.bin_enter_count = 0U;
  s_sensors.bin_exit_count = 0U;
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

void Sensors_SetColorDebugEnabled(uint8_t enabled)
{
  s_sensors.manual_color_debug_enabled = (uint8_t)((enabled != 0U) ? 1U : 0U);
  s_sensors.last_debug_tick = 0U;
}

void Sensors_SetAutoColorDebugEnabled(uint8_t enabled)
{
  s_sensors.auto_color_debug_enabled = (uint8_t)((enabled != 0U) ? 1U : 0U);
  s_sensors.last_debug_tick = 0U;
}

uint8_t Sensors_IsColorDebugEnabled(void)
{
  return s_sensors.manual_color_debug_enabled;
}
