#ifndef __MOTOR_SERVO_H__
#define __MOTOR_SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SERVO_ID_DOOR_1          1
#define SERVO_ID_DOOR_2          2

#define SERVO_MIN_PULSE_US       500U
#define SERVO_MAX_PULSE_US       2500U

void Servo_Init(void);
void Servo_SetAngle(int id, int angle);
void Servo_SetDoorAngles(int angle_1, int angle_2);
void Servo_SetDoorsSameAngle(int angle);

void Servo_Open(void);
void Servo_Close(void);
void Servo_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_SERVO_H__ */
