#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "comm.h"

void Sensors_Init(void);
void Sensors_Update(void);
void Sensors_SetTarget(uint8_t bin_id, Comm_Color_t color);
void Sensors_ClearTarget(void);
uint8_t Sensors_IsBinConfirmed(uint8_t bin_id);
uint8_t Sensors_IsHomeConfirmed(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
