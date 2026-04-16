#include "sensors.h"

#include "TCS34725.h"
#include "debug_io.h"

typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t tol_r;
  uint8_t tol_g;
  uint8_t tol_b;
} Sensors_ColorWindow_t;

typedef struct
{
  uint8_t tcs_ready;
  uint8_t target_bin;
  Comm_Color_t target_color;
  uint8_t bin_confirmed;
  uint8_t home_confirmed;
  uint8_t last_r;
  uint8_t last_g;
  uint8_t last_b;
  uint32_t last_update_tick;
  uint32_t last_debug_tick;
} Sensors_Context_t;

static Sensors_Context_t s_sensors;

static const Sensors_ColorWindow_t kColorRed = {220U, 70U, 70U, 70U, 60U, 60U};
static const Sensors_ColorWindow_t kColorGreen = {80U, 210U, 80U, 60U, 70U, 60U};
static const Sensors_ColorWindow_t kColorBlue = {70U, 120U, 225U, 60U, 70U, 70U};
static const Sensors_ColorWindow_t kColorYellow = {225U, 200U, 70U, 70U, 70U, 70U};

static const char *Sensors_ColorName(Comm_Color_t color)
{
  switch (color)
  {
    case COMM_COLOR_RED: return "RED";
    case COMM_COLOR_GREEN: return "GREEN";
    case COMM_COLOR_BLUE: return "BLUE";
    case COMM_COLOR_YELLOW: return "YELLOW";
    default: return "NONE";
  }
}

static const Sensors_ColorWindow_t *Sensors_GetColorWindow(Comm_Color_t color)
{
  switch (color)
  {
    case COMM_COLOR_RED:
      return &kColorRed;
    case COMM_COLOR_GREEN:
      return &kColorGreen;
    case COMM_COLOR_BLUE:
      return &kColorBlue;
    case COMM_COLOR_YELLOW:
      return &kColorYellow;
    default:
      return NULL;
  }
}

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

static uint8_t Sensors_IsColorMatch(uint8_t r, uint8_t g, uint8_t b, const Sensors_ColorWindow_t *window)
{
  if (window == NULL)
  {
    return 0U;
  }

  return (uint8_t)(Sensors_IsChannelMatch(r, window->r, window->tol_r) != 0U &&
                   Sensors_IsChannelMatch(g, window->g, window->tol_g) != 0U &&
                   Sensors_IsChannelMatch(b, window->b, window->tol_b) != 0U);
}

void Sensors_Init(void)
{
  s_sensors.tcs_ready = (uint8_t)((TCS34725_Init() == 0U) ? 1U : 0U);
  s_sensors.target_bin = 0U;
  s_sensors.target_color = COMM_COLOR_UNKNOWN;
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
  const Sensors_ColorWindow_t *target_window;

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

  target_window = Sensors_GetColorWindow(s_sensors.target_color);
  s_sensors.home_confirmed = Sensors_IsWhiteHomeMatch(raw_rgb, s_sensors.last_r, s_sensors.last_g, s_sensors.last_b);

  if ((s_sensors.target_bin != 0U) && (target_window != NULL))
  {
    s_sensors.bin_confirmed = Sensors_IsColorMatch(s_sensors.last_r, s_sensors.last_g, s_sensors.last_b, target_window);
  }
  else
  {
    s_sensors.bin_confirmed = 0U;
  }

  if ((HAL_GetTick() - s_sensors.last_debug_tick) >= 250U)
  {
    s_sensors.last_debug_tick = HAL_GetTick();
    DEBUG_PRINT("SENS C=%u RGB=%u,%u,%u tgt=%u/%s bin=%u home=%u", raw_rgb.C, s_sensors.last_r, s_sensors.last_g, s_sensors.last_b, s_sensors.target_bin, Sensors_ColorName(s_sensors.target_color), s_sensors.bin_confirmed, s_sensors.home_confirmed);
  }
}

void Sensors_SetTarget(uint8_t bin_id, Comm_Color_t color)
{
  s_sensors.target_bin = bin_id;
  s_sensors.target_color = color;
  s_sensors.bin_confirmed = 0U;
}

void Sensors_ClearTarget(void)
{
  s_sensors.target_bin = 0U;
  s_sensors.target_color = COMM_COLOR_UNKNOWN;
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


