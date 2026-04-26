#ifndef __MOTOR_SERVO_H__
#define __MOTOR_SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SERVO_ID_DOOR_1          1
#define SERVO_ID_DOOR_2          2

/*
 * MG90S 作为标准位置舵机使用时，理论常见范围是 500~2500us，
 * 但很多实物在极限两端更容易顶死、发热或持续嗡鸣。
 * 这里默认收窄到更保守的安全范围，后续如果机构需要更大行程再微调。
 */
#define SERVO_MIN_PULSE_US       600U
#define SERVO_MAX_PULSE_US       2400U

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
