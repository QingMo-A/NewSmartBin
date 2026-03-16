#ifndef __MOTOR_L9110S_H__
#define __MOTOR_L9110S_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} MotorDirection_t;

void Motor_Init(void);
void Motor_Left(int speed);
void Motor_Right(int speed);
void Motor_Run(int left_speed, int right_speed);
void Motor_RunForward(int speed);
void Motor_Stop(void);


#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_L9110S_H__ */
