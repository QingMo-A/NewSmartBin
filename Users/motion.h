#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Motion_Init(void);
void Motion_Update(void);
void Motion_MoveToBin(uint8_t bin_id);
uint8_t Motion_IsAtTarget(void);
void Motion_ReturnHome(void);
uint8_t Motion_IsAtHome(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_H */
