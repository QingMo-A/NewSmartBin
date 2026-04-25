#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "comm.h"

void Sensors_Init(void);
void Sensors_Update(void);
void Sensors_SetTarget(uint8_t bin_id, const Comm_ColorSpec_t *color_spec);
void Sensors_SetHomeColor(const Comm_ColorSpec_t *color_spec);
void Sensors_ClearTarget(void);
uint8_t Sensors_IsBinConfirmed(uint8_t bin_id);
uint8_t Sensors_IsHomeConfirmed(void);
void Sensors_SetColorDebugEnabled(uint8_t enabled);
uint8_t Sensors_IsColorDebugEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
