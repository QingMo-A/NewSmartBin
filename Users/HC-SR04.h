#ifndef __HC_SR04_H__
#define __HC_SR04_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

void HCSR04_Init(void);
bool HCSR04_Measure(void);
float HCSR04_GetDistanceCm(void);
uint32_t HCSR04_GetPulseWidthUs(void);
bool HCSR04_IsDataValid(void);

#ifdef __cplusplus
}
#endif

#endif /* __HC_SR04_H__ */
