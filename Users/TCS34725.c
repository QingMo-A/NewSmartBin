#include "TCS34725.h"
#include "i2c.h"

static const float kIntegrationCyclesMin = 1.0f;
static const float kIntegrationCyclesMax = 256.0f;
static const float kIntegrationTimeMsMin = 2.4f;
static const float kIntegrationTimeMsMax = 2.4f * 256.0f;
static const uint32_t kStatusTimeoutMs = 100U;
static const uint32_t kI2cTimeoutMs = 100U;

static uint8_t TCS34725_NormalizeAtime(float ms)
{
    if (ms < kIntegrationTimeMsMin)
    {
        ms = kIntegrationTimeMsMin;
    }
    if (ms > kIntegrationTimeMsMax)
    {
        ms = kIntegrationTimeMsMax;
    }

    return (uint8_t)(256.0f - (ms / kIntegrationTimeMsMin));
}

static RGB TCS34725_NormalizeRgb(RGB rgb)
{
    uint32_t max_value = rgb.R;
    float scale = 1.0f;

    if (rgb.G > max_value)
    {
        max_value = rgb.G;
    }
    if (rgb.B > max_value)
    {
        max_value = rgb.B;
    }

    if (max_value > 255U)
    {
        scale = (float)max_value / 255.0f;
        rgb.R = (uint16_t)((float)rgb.R / scale);
        rgb.G = (uint16_t)((float)rgb.G / scale);
        rgb.B = (uint16_t)((float)rgb.B / scale);
    }

    if (rgb.R > 30U)
    {
        rgb.R = (uint16_t)(rgb.R - 30U);
    }
    if (rgb.G > 30U)
    {
        rgb.G = (uint16_t)(rgb.G - 30U);
    }
    if (rgb.B > 30U)
    {
        rgb.B = (uint16_t)(rgb.B - 30U);
    }

    rgb.R = (uint16_t)((uint32_t)rgb.R * 255U / 225U);
    rgb.G = (uint16_t)((uint32_t)rgb.G * 255U / 225U);
    rgb.B = (uint16_t)((uint32_t)rgb.B * 255U / 225U);

    if (rgb.R > 255U)
    {
        rgb.R = 255U;
    }
    if (rgb.G > 255U)
    {
        rgb.G = 255U;
    }
    if (rgb.B > 255U)
    {
        rgb.B = 255U;
    }

    return rgb;
}

uint8_t TCS34725_ReadWord(uint8_t *pBuffer, uint8_t read_addr, uint16_t size)
{
    HAL_StatusTypeDef hal_status;

    if ((pBuffer == NULL) || (size == 0U))
    {
        return 1U;
    }

    hal_status = HAL_I2C_Mem_Read(
        &hi2c1,
        TCS34725_I2C_ADDR,
        (uint16_t)(TCS34725_CMD_BIT | read_addr),
        I2C_MEMADD_SIZE_8BIT,
        pBuffer,
        size,
        kI2cTimeoutMs);

    return (hal_status == HAL_OK) ? 0U : 1U;
}

uint8_t TCS34725_WriteByte(uint8_t addr, uint8_t data)
{
    HAL_StatusTypeDef hal_status;

    hal_status = HAL_I2C_Mem_Write(
        &hi2c1,
        TCS34725_I2C_ADDR,
        (uint16_t)(TCS34725_CMD_BIT | addr),
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1U,
        kI2cTimeoutMs);

    return (hal_status == HAL_OK) ? 0U : 1U;
}

uint8_t TCS34725_Init(void)
{
    uint8_t id = 0U;

    if (TCS34725_ReadWord(&id, TCS34725_ID, 1U) != 0U)
    {
        return 1U;
    }

    if ((id != 0x44U) && (id != 0x4DU))
    {
        return 1U;
    }

    if (TCS34725_WriteByte(TCS34725_ENABLE, TCS34725_ENABLE_PON) != 0U)
    {
        return 1U;
    }
    HAL_Delay(3U);

    if (TCS34725_WriteByte(TCS34725_ATIME, TCS34725_NormalizeAtime(50.0f)) != 0U)
    {
        return 1U;
    }
    if (TCS34725_WriteByte(TCS34725_PERS, TCS34725_PERS_NONE) != 0U)
    {
        return 1U;
    }
    if (TCS34725_WriteByte(TCS34725_CONTROL, TCS34725_GAIN_60X) != 0U)
    {
        return 1U;
    }
    if (TCS34725_WriteByte(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN) != 0U)
    {
        return 1U;
    }

    return 0U;
}

void integrationTime(float ms)
{
    (void)kIntegrationCyclesMin;
    (void)kIntegrationCyclesMax;
    (void)TCS34725_WriteByte(TCS34725_ATIME, TCS34725_NormalizeAtime(ms));
}

RGB TCS34725_Get_RGBData(void)
{
    RGB temp = {0};
    uint8_t status = 0U;
    uint8_t raw_data[8] = {0};
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < kStatusTimeoutMs)
    {
        if (TCS34725_ReadWord(&status, TCS34725_STATUS, 1U) != 0U)
        {
            return temp;
        }

        if ((status & TCS34725_STATUS_AVALID) != 0U)
        {
            break;
        }
    }

    if ((status & TCS34725_STATUS_AVALID) == 0U)
    {
        return temp;
    }

    if (TCS34725_ReadWord(raw_data, (uint8_t)(TCS34725_CMD_AUTO_INC | TCS34725_CDATAL), 8U) != 0U)
    {
        return temp;
    }

    temp.C = (uint16_t)(((uint16_t)raw_data[1] << 8) | raw_data[0]);
    temp.R = (uint16_t)(((uint16_t)raw_data[3] << 8) | raw_data[2]);
    temp.G = (uint16_t)(((uint16_t)raw_data[5] << 8) | raw_data[4]);
    temp.B = (uint16_t)(((uint16_t)raw_data[7] << 8) | raw_data[6]);

    return temp;
}

uint32_t TCS34725_GetRGB888(RGB rgb)
{
    rgb = TCS34725_NormalizeRgb(rgb);
    return ((uint32_t)rgb.R << 16) | ((uint32_t)rgb.G << 8) | (uint32_t)rgb.B;
}

uint16_t TCS34725_GetRGB565(RGB rgb)
{
    rgb = TCS34725_NormalizeRgb(rgb);
    return (uint16_t)((((uint16_t)rgb.R & 0xF8U) << 8)
        | (((uint16_t)rgb.G & 0xFCU) << 3)
        | (((uint16_t)rgb.B & 0xF8U) >> 3));
}
