#include "Motor_Servo.h"
#include "tim.h"

static int Servo_ClampAngle(int angle)
{
    if (angle < 0)
    {
        return 0;
    }

    if (angle > 180)
    {
        return 180;
    }

    return angle;
}

static int Servo_AngleToPulseUs(int angle)
{
    angle = Servo_ClampAngle(angle);
    return SERVO_MIN_PULSE_US + (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
}

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void Servo_SetAngle(int id, int angle)
{
    int pulse_us;
    int compare;

    pulse_us = Servo_AngleToPulseUs(angle);
    compare = pulse_us;

    if (id == SERVO_ID_DOOR_1)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
    }
    else if (id == SERVO_ID_DOOR_2)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare);
    }
}

void Servo_SetDoorAngles(int angle_1, int angle_2)
{
    Servo_SetAngle(SERVO_ID_DOOR_1, angle_1);
    Servo_SetAngle(SERVO_ID_DOOR_2, angle_2);
}

void Servo_SetDoorsSameAngle(int angle)
{
    Servo_SetDoorAngles(angle, angle);
}

void Servo_Open(void)
{
    Servo_SetDoorsSameAngle(0);
}

void Servo_Close(void)
{
    Servo_SetDoorsSameAngle(90);
}

void Servo_Test(void)
{
    Servo_SetDoorsSameAngle(0);
    HAL_Delay(1000);
    Servo_SetDoorsSameAngle(45);
    HAL_Delay(1000);
    Servo_SetDoorsSameAngle(90);
    HAL_Delay(1000);
    Servo_SetDoorsSameAngle(135);
    HAL_Delay(1000);
    Servo_SetDoorsSameAngle(180);
}
