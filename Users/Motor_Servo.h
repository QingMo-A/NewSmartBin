#ifndef __MOTOR_SERVO_H__
#define __MOTOR_SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SERVO_MIN_PULSE_US 500U
#define SERVO_MAX_PULSE_US 2500U

void Servo_Init(void);
void Servo_SetAngle(int id, int angle);


#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_SERVO_H__ */
