#include "Motor_Servo.h"
#include "tim.h"

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // Servo_Close();
}

void Servo_SetAngle(int id, int angle)
{
    int pulse_us;
    int compare;

    if (angle < 0)
    {
        angle = 0;
    }

    if (angle > 180)
    {
        angle = 180;
    }

    pulse_us = SERVO_MIN_PULSE_US + (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
    compare = pulse_us / 2;

    if (id == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
    }
    else if (id == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare);
    }
}

void Servo_Open(void)
{
    Servo_SetAngle(SERVO_ID_DOOR, SERVO_DOOR_OPEN_ANGLE);
}

void Servo_Close(void)
{
    Servo_SetAngle(SERVO_ID_DOOR, SERVO_DOOR_CLOSE_ANGLE);
}

void Servo_Test(void) {
    Servo_SetAngle(1,0);
    HAL_Delay(1000);
    Servo_SetAngle(1,45);
    HAL_Delay(1000);
    Servo_SetAngle(1,90);
    HAL_Delay(1000);
    Servo_SetAngle(1,135);
    HAL_Delay(1000);
    Servo_SetAngle(1,180);
}
