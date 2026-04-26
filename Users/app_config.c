#include "app_config.h"

#include <stdio.h>

#define APP_CONFIG_MS_MAX              120000UL
#define APP_CONFIG_GATE_SETTLE_MS_MAX  10000UL
#define APP_CONFIG_MOTION_SPEED_MAX    90U
#define APP_CONFIG_GATE_ANGLE_MAX      180U

static App_Config_t s_app_config;

static const App_Config_t s_app_config_defaults =
{
  20000UL,
  1000UL,
  3000UL,
  1500UL,
  1000UL,
  20000UL,
  200UL,
  80U,
  20U,
  80U,
  600UL,
  600UL
};

static uint8_t AppConfig_IsMsValid(uint32_t value)
{
  return (uint8_t)(value <= APP_CONFIG_MS_MAX);
}

static uint8_t AppConfig_IsGateSettleValid(uint32_t value)
{
  return (uint8_t)(value <= APP_CONFIG_GATE_SETTLE_MS_MAX);
}

static uint8_t AppConfig_IsMotionSpeedValid(uint8_t value)
{
  return (uint8_t)(value <= APP_CONFIG_MOTION_SPEED_MAX);
}

static uint8_t AppConfig_IsGateAngleValid(uint8_t value)
{
  return (uint8_t)(value <= APP_CONFIG_GATE_ANGLE_MAX);
}

void AppConfig_Init(void)
{
  s_app_config = s_app_config_defaults;
}

const App_Config_t *AppConfig_Get(void)
{
  return &s_app_config;
}

HAL_StatusTypeDef AppConfig_Apply(const App_Config_t *config, uint32_t mask)
{
  if ((config == NULL) || ((mask & ~APP_CONFIG_MASK_ALL) != 0UL))
  {
    return HAL_ERROR;
  }

  if (((mask & APP_CONFIG_MASK_MOVE_TIMEOUT_MS) != 0UL) &&
      (AppConfig_IsMsValid(config->move_timeout_ms) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_PREOPEN_DELAY_MS) != 0UL) &&
      (AppConfig_IsMsValid(config->preopen_delay_ms) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_GATE_TIMEOUT_MS) != 0UL) &&
      (AppConfig_IsMsValid(config->gate_timeout_ms) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_DROP_WAIT_MS) != 0UL) &&
      (AppConfig_IsMsValid(config->drop_wait_ms) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_POSTCLOSE_DELAY_MS) != 0UL) &&
      (AppConfig_IsMsValid(config->postclose_delay_ms) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_RETURN_HOME_TIMEOUT_MS) != 0UL) &&
      (AppConfig_IsMsValid(config->return_home_timeout_ms) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_ERROR_BLINK_MS) != 0UL) &&
      (AppConfig_IsMsValid(config->error_blink_ms) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_MOTION_SPEED) != 0UL) &&
      (AppConfig_IsMotionSpeedValid(config->motion_speed) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_GATE_OPEN_ANGLE) != 0UL) &&
      (AppConfig_IsGateAngleValid(config->gate_open_angle) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_GATE_CLOSE_ANGLE) != 0UL) &&
      (AppConfig_IsGateAngleValid(config->gate_close_angle) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_GATE_OPEN_SETTLE_MS) != 0UL) &&
      (AppConfig_IsGateSettleValid(config->gate_open_settle_ms) == 0U))
  {
    return HAL_ERROR;
  }
  if (((mask & APP_CONFIG_MASK_GATE_CLOSE_SETTLE_MS) != 0UL) &&
      (AppConfig_IsGateSettleValid(config->gate_close_settle_ms) == 0U))
  {
    return HAL_ERROR;
  }

  if ((mask & APP_CONFIG_MASK_MOVE_TIMEOUT_MS) != 0UL)
  {
    s_app_config.move_timeout_ms = config->move_timeout_ms;
  }
  if ((mask & APP_CONFIG_MASK_PREOPEN_DELAY_MS) != 0UL)
  {
    s_app_config.preopen_delay_ms = config->preopen_delay_ms;
  }
  if ((mask & APP_CONFIG_MASK_GATE_TIMEOUT_MS) != 0UL)
  {
    s_app_config.gate_timeout_ms = config->gate_timeout_ms;
  }
  if ((mask & APP_CONFIG_MASK_DROP_WAIT_MS) != 0UL)
  {
    s_app_config.drop_wait_ms = config->drop_wait_ms;
  }
  if ((mask & APP_CONFIG_MASK_POSTCLOSE_DELAY_MS) != 0UL)
  {
    s_app_config.postclose_delay_ms = config->postclose_delay_ms;
  }
  if ((mask & APP_CONFIG_MASK_RETURN_HOME_TIMEOUT_MS) != 0UL)
  {
    s_app_config.return_home_timeout_ms = config->return_home_timeout_ms;
  }
  if ((mask & APP_CONFIG_MASK_ERROR_BLINK_MS) != 0UL)
  {
    s_app_config.error_blink_ms = config->error_blink_ms;
  }
  if ((mask & APP_CONFIG_MASK_MOTION_SPEED) != 0UL)
  {
    s_app_config.motion_speed = config->motion_speed;
  }
  if ((mask & APP_CONFIG_MASK_GATE_OPEN_ANGLE) != 0UL)
  {
    s_app_config.gate_open_angle = config->gate_open_angle;
  }
  if ((mask & APP_CONFIG_MASK_GATE_CLOSE_ANGLE) != 0UL)
  {
    s_app_config.gate_close_angle = config->gate_close_angle;
  }
  if ((mask & APP_CONFIG_MASK_GATE_OPEN_SETTLE_MS) != 0UL)
  {
    s_app_config.gate_open_settle_ms = config->gate_open_settle_ms;
  }
  if ((mask & APP_CONFIG_MASK_GATE_CLOSE_SETTLE_MS) != 0UL)
  {
    s_app_config.gate_close_settle_ms = config->gate_close_settle_ms;
  }

  return HAL_OK;
}

HAL_StatusTypeDef AppConfig_Format(char *buffer, uint16_t buffer_len)
{
  int len;

  if ((buffer == NULL) || (buffer_len == 0U))
  {
    return HAL_ERROR;
  }

  len = snprintf(buffer,
                 buffer_len,
                 "[$CFG:MT=%lu,PD=%lu,GT=%lu,DW=%lu,PC=%lu,RH=%lu,EB=%lu,SP=%u,OA=%u,CA=%u,OS=%lu,CS=%lu]",
                 (unsigned long)s_app_config.move_timeout_ms,
                 (unsigned long)s_app_config.preopen_delay_ms,
                 (unsigned long)s_app_config.gate_timeout_ms,
                 (unsigned long)s_app_config.drop_wait_ms,
                 (unsigned long)s_app_config.postclose_delay_ms,
                 (unsigned long)s_app_config.return_home_timeout_ms,
                 (unsigned long)s_app_config.error_blink_ms,
                 s_app_config.motion_speed,
                 s_app_config.gate_open_angle,
                 s_app_config.gate_close_angle,
                 (unsigned long)s_app_config.gate_open_settle_ms,
                 (unsigned long)s_app_config.gate_close_settle_ms);

  if ((len <= 0) || (len >= (int)buffer_len))
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}
