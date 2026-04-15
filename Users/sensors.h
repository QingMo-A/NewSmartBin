#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Sensors_Init(void);
void Sensors_Update(void);
uint8_t Sensors_IsBinConfirmed(uint8_t bin_id);
uint8_t Sensors_IsHomeConfirmed(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
