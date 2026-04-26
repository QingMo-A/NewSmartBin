#ifndef GATE_H
#define GATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Gate_Init(void);
void Gate_Update(void);
void Gate_Open(void);
void Gate_Close(void);
uint8_t Gate_IsBusy(void);
uint8_t Gate_IsOpenDone(void);
uint8_t Gate_IsCloseDone(void);

#ifdef __cplusplus
}
#endif

#endif /* GATE_H */
