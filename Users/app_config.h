#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  uint32_t move_timeout_ms;
  uint32_t preopen_delay_ms;
  uint32_t gate_timeout_ms;
  uint32_t drop_wait_ms;
  uint32_t postclose_delay_ms;
  uint32_t return_home_timeout_ms;
  uint32_t error_blink_ms;
  uint8_t motion_speed;
  uint8_t gate1_open_angle;
  uint8_t gate1_close_angle;
  uint8_t gate2_open_angle;
  uint8_t gate2_close_angle;
  uint32_t gate_open_settle_ms;
  uint32_t gate_close_settle_ms;
} App_Config_t;

#define APP_CONFIG_MASK_MOVE_TIMEOUT_MS        (1UL << 0)
#define APP_CONFIG_MASK_PREOPEN_DELAY_MS       (1UL << 1)
#define APP_CONFIG_MASK_GATE_TIMEOUT_MS        (1UL << 2)
#define APP_CONFIG_MASK_DROP_WAIT_MS           (1UL << 3)
#define APP_CONFIG_MASK_POSTCLOSE_DELAY_MS     (1UL << 4)
#define APP_CONFIG_MASK_RETURN_HOME_TIMEOUT_MS (1UL << 5)
#define APP_CONFIG_MASK_ERROR_BLINK_MS         (1UL << 6)
#define APP_CONFIG_MASK_MOTION_SPEED           (1UL << 7)
#define APP_CONFIG_MASK_GATE1_OPEN_ANGLE       (1UL << 8)
#define APP_CONFIG_MASK_GATE1_CLOSE_ANGLE      (1UL << 9)
#define APP_CONFIG_MASK_GATE2_OPEN_ANGLE       (1UL << 10)
#define APP_CONFIG_MASK_GATE2_CLOSE_ANGLE      (1UL << 11)
#define APP_CONFIG_MASK_GATE_OPEN_SETTLE_MS    (1UL << 12)
#define APP_CONFIG_MASK_GATE_CLOSE_SETTLE_MS   (1UL << 13)

#define APP_CONFIG_MASK_ALL                    (0x00003FFFUL)

void AppConfig_Init(void);
const App_Config_t *AppConfig_Get(void);
HAL_StatusTypeDef AppConfig_Apply(const App_Config_t *config, uint32_t mask);
HAL_StatusTypeDef AppConfig_Format(char *buffer, uint16_t buffer_len);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
