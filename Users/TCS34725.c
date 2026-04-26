#include "TCS34725.h"
#include "i2c.h"

static const float kIntegrationCyclesMin = 1.0f;
static const float kIntegrationCyclesMax = 256.0f;
static const float kIntegrationTimeMsMin = 2.4f;
static const float kIntegrationTimeMsMax = 2.4f * 256.0f;
static const uint32_t kStatusTimeoutMs = 100U;
static const uint32_t kI2cTimeoutMs = 100U;

#define TCS34725_CAL_MODE_WHITE_BALANCE 1U
#define TCS34725_CAL_MODE_BLACK_WHITE   2U

/*
 * 校正模式说明：
 * 1. WHITE_BALANCE：先按 (channel / C) * 255 做白光归一化，再乘以三个增益。
 * 2. BLACK_WHITE：按黑白样本对每个通道做 0~255 线性映射。
 *
 * 默认先启用白光归一化，这样比原来那套“减 30 再拉伸”更稳定，也更容易后续细调。
 * 如果后面已经量出了黑白样本原始值，再把模式切到 BLACK_WHITE 即可。
 */
#define TCS34725_CAL_MODE               TCS34725_CAL_MODE_WHITE_BALANCE

/* 白光校正增益。先保持 1.0f，后面可按白样本改成 255 / 当前白样本值。 */
#define TCS34725_WHITE_GAIN_R           1.0f
#define TCS34725_WHITE_GAIN_G           1.0f
#define TCS34725_WHITE_GAIN_B           1.0f

/*
 * 黑白双点校正参数。
 * 只有当 TCS34725_CAL_MODE 切到 TCS34725_CAL_MODE_BLACK_WHITE 时才会生效。
 * 这里先给成“未校正”的占位值，等你后面采到黑白样本原始值再替换。
 */
#define TCS34725_BLACK_REF_R            0U
#define TCS34725_BLACK_REF_G            0U
#define TCS34725_BLACK_REF_B            0U
#define TCS34725_WHITE_REF_R            255U
#define TCS34725_WHITE_REF_G            255U
#define TCS34725_WHITE_REF_B            255U

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

static uint16_t TCS34725_ClampToByte(float value)
{
    if (value <= 0.0f)
    {
        return 0U;
    }
    if (value >= 255.0f)
    {
        return 255U;
    }

    return (uint16_t)(value + 0.5f);
}

static uint16_t TCS34725_ApplyLinearChannel(uint16_t value, uint16_t black_ref, uint16_t white_ref)
{
    int32_t scaled;

    if (white_ref <= black_ref)
    {
        return 0U;
    }

    scaled = ((int32_t)(value - black_ref) * 255) / (int32_t)(white_ref - black_ref);
    if (scaled <= 0)
    {
        return 0U;
    }
    if (scaled >= 255)
    {
        return 255U;
    }

    return (uint16_t)scaled;
}

RGB TCS34725_GetCalibratedRgb(RGB raw)
{
    RGB corrected = {0};

    corrected.C = raw.C;

    if (raw.C == 0U)
    {
        return corrected;
    }

#if (TCS34725_CAL_MODE == TCS34725_CAL_MODE_BLACK_WHITE)
    corrected.R = TCS34725_ApplyLinearChannel(raw.R, TCS34725_BLACK_REF_R, TCS34725_WHITE_REF_R);
    corrected.G = TCS34725_ApplyLinearChannel(raw.G, TCS34725_BLACK_REF_G, TCS34725_WHITE_REF_G);
    corrected.B = TCS34725_ApplyLinearChannel(raw.B, TCS34725_BLACK_REF_B, TCS34725_WHITE_REF_B);
#else
    corrected.R = TCS34725_ClampToByte((((float)raw.R / (float)raw.C) * 255.0f) * TCS34725_WHITE_GAIN_R);
    corrected.G = TCS34725_ClampToByte((((float)raw.G / (float)raw.C) * 255.0f) * TCS34725_WHITE_GAIN_G);
    corrected.B = TCS34725_ClampToByte((((float)raw.B / (float)raw.C) * 255.0f) * TCS34725_WHITE_GAIN_B);
#endif

    return corrected;
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
    rgb = TCS34725_GetCalibratedRgb(rgb);
    return ((uint32_t)rgb.R << 16) | ((uint32_t)rgb.G << 8) | (uint32_t)rgb.B;
}

uint16_t TCS34725_GetRGB565(RGB rgb)
{
    rgb = TCS34725_GetCalibratedRgb(rgb);
    return (uint16_t)((((uint16_t)rgb.R & 0xF8U) << 8)
        | (((uint16_t)rgb.G & 0xFCU) << 3)
        | (((uint16_t)rgb.B & 0xF8U) >> 3));
}
