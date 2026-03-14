#ifndef __SCCB_H__
#define __SCCB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SCCB_ADDR_OV7725_WRITE 0x42U
#define SCCB_ADDR_OV7725_READ  0x43U

void SCCB_Init(void);
HAL_StatusTypeDef SCCB_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef SCCB_ReadReg(uint8_t reg, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* __SCCB_H__ */
