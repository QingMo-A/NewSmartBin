#include "Motor_L9110S.h"
#include "gpio.h"
#include "tim.h"

static uint16_t Motor_Speed2PWM(int speed)
{
    if (speed < 0)
    {
        speed = -speed;
    }

    if (speed > 90)
    {
        speed = 90;
    }

    return (uint16_t)speed;
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    Motor_Stop();
}

void Motor_Left(int speed)
{
    uint16_t pwm = Motor_Speed2PWM(speed);

    if (speed > 0)
    {
        HAL_GPIO_WritePin(A_IB_GPIO_Port, A_IB_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(A_IB_GPIO_Port, A_IB_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,100 - pwm);
    }
    else
    {
        HAL_GPIO_WritePin(A_IB_GPIO_Port, A_IB_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
}

void Motor_Right(int speed)
{
    uint16_t pwm = Motor_Speed2PWM(speed);

    if (speed > 0)
    {
        HAL_GPIO_WritePin(B_IB_GPIO_Port, B_IB_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(B_IB_GPIO_Port, B_IB_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100 - pwm);
    }
    else
    {
        HAL_GPIO_WritePin(B_IB_GPIO_Port, B_IB_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

//自定义速度，负数倒转，正数正转
void Motor_Run(int left_speed, int right_speed)
{
    Motor_Left(left_speed);
    Motor_Right(right_speed);
}

void Motor_RunForward(int speed)
{
    Motor_Left(speed);
    Motor_Right(speed);
}

void Motor_Stop(void)
{
    HAL_GPIO_WritePin(A_IB_GPIO_Port, A_IB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_IB_GPIO_Port, B_IB_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}
