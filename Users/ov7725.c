#include "ov7725.h"

#include "dcmi.h"
#include "sccb.h"

typedef struct
{
  uint8_t reg;
  uint8_t value;
} OV7725_RegValueTypeDef;

enum
{
  OV7725_REG_GAIN      = 0x00,
  OV7725_REG_BLUE      = 0x01,
  OV7725_REG_RED       = 0x02,
  OV7725_REG_GREEN     = 0x03,
  OV7725_REG_COM2      = 0x09,
  OV7725_REG_PID       = 0x0A,
  OV7725_REG_VER       = 0x0B,
  OV7725_REG_COM3      = 0x0C,
  OV7725_REG_COM4      = 0x0D,
  OV7725_REG_COM5      = 0x0E,
  OV7725_REG_COM6      = 0x0F,
  OV7725_REG_AEC       = 0x10,
  OV7725_REG_CLKRC     = 0x11,
  OV7725_REG_COM7      = 0x12,
  OV7725_REG_COM8      = 0x13,
  OV7725_REG_COM10     = 0x15,
  OV7725_REG_HSTART    = 0x17,
  OV7725_REG_HSIZE     = 0x18,
  OV7725_REG_VSTRT     = 0x19,
  OV7725_REG_VSIZE     = 0x1A,
  OV7725_REG_MIDH      = 0x1C,
  OV7725_REG_MIDL      = 0x1D,
  OV7725_REG_BDBASE    = 0x22,
  OV7725_REG_AEW       = 0x24,
  OV7725_REG_AEB       = 0x25,
  OV7725_REG_VPT       = 0x26,
  OV7725_REG_HOUTSIZE  = 0x29,
  OV7725_REG_EXHCH     = 0x2A,
  OV7725_REG_EXHCL     = 0x2B,
  OV7725_REG_VOUTSIZE  = 0x2C,
  OV7725_REG_DSP_CTRL3 = 0x66,
  OV7725_REG_SCAL0     = 0xA4,
  OV7725_REG_SLOP      = 0xAC,
  OV7725_REG_TGT_B     = 0x42
};

static const OV7725_RegValueTypeDef ov7725_qvga_rgb565_regs[] =
{
  {OV7725_REG_COM3, 0xD0},
  {OV7725_REG_CLKRC, 0x00},
  {OV7725_REG_COM7, 0x46},
  {OV7725_REG_HSTART, 0x3F},
  {OV7725_REG_HSIZE, 0x50},
  {OV7725_REG_VSTRT, 0x03},
  {OV7725_REG_VSIZE, 0x78},
  {0x1B, 0x00},
  {OV7725_REG_HOUTSIZE, 0x50},
  {OV7725_REG_VOUTSIZE, 0x78},
  {OV7725_REG_EXHCH, 0x00},
  {OV7725_REG_COM10, 0x02},
  {OV7725_REG_TGT_B, 0x7F},
  {0x4C, 0x00},
  {0x4D, 0x09},
  {0x42, 0x7F},
  {OV7725_REG_EXHCL, 0x00},
  {OV7725_REG_COM6, 0xC5},
  {0x7E, 0x0C},
  {0x7F, 0x16},
  {0x80, 0x2A},
  {0x81, 0x4E},
  {0x82, 0x61},
  {0x83, 0x6F},
  {0x84, 0x7B},
  {0x85, 0x86},
  {0x86, 0x8E},
  {0x87, 0x97},
  {0x88, 0xA4},
  {0x89, 0xAF},
  {0x8A, 0xC5},
  {0x8B, 0xD7},
  {0x8C, 0xE8},
  {0x8D, 0x20},
  {OV7725_REG_COM5, 0x65},
  {OV7725_REG_GAIN, 0x00},
  {OV7725_REG_BLUE, 0x80},
  {OV7725_REG_RED, 0x80},
  {OV7725_REG_GREEN, 0x00},
  {OV7725_REG_COM8, 0xF0},
  {0x05, 0x05},
  {0x06, 0xC3},
  {0x07, 0x00},
  {OV7725_REG_AEC, 0x00},
  {0x16, 0x03},
  {OV7725_REG_COM2, 0x03},
  {OV7725_REG_COM4, 0x81},
  {0x23, 0x00},
  {OV7725_REG_AEW, 0x40},
  {OV7725_REG_AEB, 0x30},
  {OV7725_REG_VPT, 0xA1},
  {OV7725_REG_BDBASE, 0xFF},
  {OV7725_REG_COM5, 0xF5},
  {OV7725_REG_COM6, 0xC5},
  {0x33, 0x00},
  {0x34, 0x08},
  {0x36, 0x2B},
  {0x38, 0x14},
  {0x39, 0x22},
  {0x3A, 0x80},
  {0x3B, 0x40},
  {0x3C, 0x80},
  {0x3D, 0x80},
  {0x3E, 0x00},
  {0x3F, 0x00},
  {0x40, 0x00},
  {0x41, 0x00},
  {0x46, 0x0B},
  {0x47, 0x22},
  {0x48, 0x9F},
  {0x4A, 0x10},
  {0x4B, 0xD4},
  {0x4E, 0x00},
  {0x4F, 0x93},
  {0x50, 0x80},
  {0x51, 0x80},
  {0x52, 0x91},
  {0x53, 0x00},
  {0x54, 0x11},
  {0x55, 0x11},
  {0x56, 0x00},
  {0x57, 0x00},
  {0x58, 0x00},
  {0x59, 0x00},
  {0x5A, 0xD8},
  {0x5B, 0x00},
  {0x5C, 0x00},
  {0x5D, 0x00},
  {0x5E, 0x00},
  {0x5F, 0x00},
  {0x60, 0x00},
  {0x61, 0x00},
  {0x62, 0x00},
  {0x63, 0x00},
  {0x64, 0x00},
  {0x65, 0x00},
  {OV7725_REG_DSP_CTRL3, 0x00},
  {0x67, 0x00},
  {0x68, 0x00},
  {0x69, 0x00},
  {0x6A, 0x02},
  {0x6B, 0x40},
  {0x6C, 0x00},
  {0x6D, 0x00},
  {0x6E, 0x10},
  {0x6F, 0x00},
  {0x70, 0x00},
  {0x71, 0x00},
  {0x72, 0x00},
  {0x73, 0x00},
  {0x74, 0x00},
  {0x75, 0x00},
  {0x76, 0x00},
  {0x77, 0x00},
  {0x78, 0x00},
  {0x79, 0x00},
  {0x7A, 0x00},
  {0x7B, 0x00},
  {0x8E, 0x00},
  {0x8F, 0x00},
  {0x90, 0x00},
  {0x91, 0x00},
  {0x92, 0x00},
  {0x93, 0x00},
  {0x94, 0x2D},
  {0x95, 0x24},
  {0x96, 0x71},
  {0x97, 0x2A},
  {0x98, 0x0A},
  {0x99, 0x11},
  {0x9A, 0x1F},
  {0x9B, 0x33},
  {0x9C, 0x5C},
  {0x9D, 0x5E},
  {0x9E, 0x8C},
  {0x9F, 0xA4},
  {0xA0, 0xC8},
  {0xA1, 0xF0},
  {0xA2, 0x03},
  {0xA3, 0xE0},
  {OV7725_REG_SCAL0, 0x8F},
  {0xA5, 0x05},
  {0xA6, 0x10},
  {0xA7, 0xF0},
  {0xA8, 0xF0},
  {0xA9, 0x00},
  {0xAA, 0x14},
  {0xAB, 0x07},
  {OV7725_REG_SLOP, 0x0A}
};

static volatile uint8_t g_ov7725_frame_ready = 0U;
static volatile uint8_t g_ov7725_capture_failed = 0U;
static volatile uint32_t g_ov7725_frame_count = 0U;
static uint32_t g_ov7725_frame_buffer[OV7725_FRAME_WORDS]
  __attribute__((section(".ARM.__at_0x24000000"), aligned(32)));

static HAL_StatusTypeDef OV7725_WriteTable(const OV7725_RegValueTypeDef *table, uint32_t count)
{
  for (uint32_t i = 0U; i < count; ++i)
  {
    if (SCCB_WriteReg(table[i].reg, table[i].value) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef OV7725_ReadID(OV7725_IdTypeDef *id)
{
  if (id == NULL)
  {
    return HAL_ERROR;
  }

  if ((SCCB_ReadReg(OV7725_REG_PID, &id->pid) != HAL_OK) ||
      (SCCB_ReadReg(OV7725_REG_VER, &id->ver) != HAL_OK) ||
      (SCCB_ReadReg(OV7725_REG_MIDH, &id->midh) != HAL_OK) ||
      (SCCB_ReadReg(OV7725_REG_MIDL, &id->midl) != HAL_OK))
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef OV7725_Init(OV7725_IdTypeDef *id)
{
  SCCB_Init();
  HAL_GPIO_WritePin(CAMERA_PWDN_GPIO_Port, CAMERA_PWDN_Pin, GPIO_PIN_RESET);
  HAL_Delay(10U);

  if (OV7725_ReadID(id) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((id->pid != OV7725_PID_VALUE) || (id->ver != OV7725_VER_VALUE))
  {
    return HAL_ERROR;
  }

  if (SCCB_WriteReg(OV7725_REG_COM7, 0x80U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(10U);

  return OV7725_WriteTable(ov7725_qvga_rgb565_regs,
                           sizeof(ov7725_qvga_rgb565_regs) / sizeof(ov7725_qvga_rgb565_regs[0]));
}

HAL_StatusTypeDef OV7725_CaptureSnapshot(uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();

  g_ov7725_frame_ready = 0U;
  g_ov7725_capture_failed = 0U;

  __HAL_DCMI_CLEAR_FLAG(&hdcmi, DCMI_FLAG_FRAMERI | DCMI_FLAG_OVRRI | DCMI_FLAG_ERRRI | DCMI_FLAG_VSYNCRI | DCMI_FLAG_LINERI);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_ERR | DCMI_IT_OVR);

  if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)g_ov7725_frame_buffer, OV7725_FRAME_WORDS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  while ((g_ov7725_frame_ready == 0U) && (g_ov7725_capture_failed == 0U))
  {
    if ((HAL_GetTick() - start_tick) > timeout_ms)
    {
      (void)HAL_DCMI_Stop(&hdcmi);
      return HAL_TIMEOUT;
    }
  }

  (void)HAL_DCMI_Stop(&hdcmi);

  if (g_ov7725_capture_failed != 0U)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

const uint32_t *OV7725_GetFrameBuffer(void)
{
  return g_ov7725_frame_buffer;
}

uint32_t OV7725_GetCapturedFrameCount(void)
{
  return g_ov7725_frame_count;
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *dcmi)
{
  if (dcmi->Instance == DCMI)
  {
    g_ov7725_frame_ready = 1U;
    g_ov7725_frame_count++;
  }
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *dcmi)
{
  if (dcmi->Instance == DCMI)
  {
    g_ov7725_capture_failed = 1U;
  }
}
