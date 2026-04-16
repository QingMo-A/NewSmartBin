#ifndef __TCS34725_H__
#define __TCS34725_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TCS34725_I2C_ADDR_7BIT       0x29U
#define TCS34725_I2C_ADDR            (TCS34725_I2C_ADDR_7BIT << 1)

#define TCS34725_CMD_BIT             0x80U
#define TCS34725_CMD_AUTO_INC        0x20U

#define TCS34725_ENABLE              0x00U
#define TCS34725_ATIME               0x01U
#define TCS34725_AILTL               0x04U
#define TCS34725_AILTH               0x05U
#define TCS34725_AIHTL               0x06U
#define TCS34725_AIHTH               0x07U
#define TCS34725_PERS                0x0CU
#define TCS34725_CONFIG              0x0DU
#define TCS34725_CONTROL             0x0FU
#define TCS34725_ID                  0x12U
#define TCS34725_STATUS              0x13U
#define TCS34725_CDATAL              0x14U
#define TCS34725_CDATAH              0x15U
#define TCS34725_RDATAL              0x16U
#define TCS34725_RDATAH              0x17U
#define TCS34725_GDATAL              0x18U
#define TCS34725_GDATAH              0x19U
#define TCS34725_BDATAL              0x1AU
#define TCS34725_BDATAH              0x1BU

#define TCS34725_ENABLE_PON          0x01U
#define TCS34725_ENABLE_AEN          0x02U
#define TCS34725_ENABLE_AIEN         0x10U

#define TCS34725_PERS_NONE           0x00U

#define TCS34725_GAIN_1X             0x00U
#define TCS34725_GAIN_4X             0x01U
#define TCS34725_GAIN_16X            0x02U
#define TCS34725_GAIN_60X            0x03U

#define TCS34725_STATUS_AVALID       0x01U
#define TCS34725_STATUS_AINT         0x10U

typedef struct
{
    uint16_t C;
    uint16_t R;
    uint16_t G;
    uint16_t B;
} RGB;

uint8_t TCS34725_ReadWord(uint8_t *pBuffer, uint8_t read_addr, uint16_t size);
uint8_t TCS34725_WriteByte(uint8_t addr, uint8_t data);
uint8_t TCS34725_Init(void);
void integrationTime(float ms);
RGB TCS34725_Get_RGBData(void);
uint32_t TCS34725_GetRGB888(RGB rgb);
uint16_t TCS34725_GetRGB565(RGB rgb);

#ifdef __cplusplus
}
#endif

#endif
