#ifndef __MOTOR_SERVO_H__
#define __MOTOR_SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#define SERVO_ID_DOOR         1

#define SERVO_DOOR_CLOSE_ANGLE  45
#define SERVO_DOOR_OPEN_ANGLE   135

#define SERVO_MIN_PULSE_US 500U
#define SERVO_MAX_PULSE_US 2500U

void Servo_Init(void);
void Servo_SetAngle(int id, int angle);

void Servo_Open(void);
void Servo_Close(void);
void Servo_Test(void);


#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_SERVO_H__ */
