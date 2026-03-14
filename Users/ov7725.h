#ifndef __OV7725_H__
#define __OV7725_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define OV7725_FRAME_WIDTH   320U
#define OV7725_FRAME_HEIGHT  240U
#define OV7725_FRAME_BYTES   (OV7725_FRAME_WIDTH * OV7725_FRAME_HEIGHT * 2U)
#define OV7725_FRAME_WORDS   (OV7725_FRAME_BYTES / 4U)
#define OV7725_PID_VALUE     0x77U
#define OV7725_VER_VALUE     0x21U

typedef struct
{
  uint8_t pid;
  uint8_t ver;
  uint8_t midh;
  uint8_t midl;
} OV7725_IdTypeDef;

HAL_StatusTypeDef OV7725_ReadID(OV7725_IdTypeDef *id);
HAL_StatusTypeDef OV7725_Init(OV7725_IdTypeDef *id);
HAL_StatusTypeDef OV7725_CaptureSnapshot(uint32_t timeout_ms);
const uint32_t *OV7725_GetFrameBuffer(void);
uint32_t OV7725_GetCapturedFrameCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __OV7725_H__ */
