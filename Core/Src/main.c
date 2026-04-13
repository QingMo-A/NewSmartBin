/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HC-SR04.h"
#include "Motor_L9110S.h"
#include "Motor_Servo.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void App_Log(const char *message);
static void BT_ProcessCommand(const char *cmd);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_Log(const char *message)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)message, (uint16_t)strlen(message), 200U);
}

static void BT_ProcessCommand(const char *cmd)
{
    if (cmd == NULL)
    {
        App_Log("cmd is NULL\r\n");
        return;
    }
    
    char dbg[64];
    snprintf(dbg, sizeof(dbg), "recv cmd: %s\r\n", cmd);
    App_Log(dbg);
    
    if (strlen(cmd) != 5)
    {
        snprintf(dbg, sizeof(dbg), "invalid cmd len=%u\r\n", (unsigned int)strlen(cmd));
        App_Log(dbg);
        return;
    }

    /* 舵机测试命令：ta000 */
    if (strncmp(cmd, "ta000", 5) == 0)
    {
        App_Log("servo test\r\n");
        Servo_Test();
        return;
    }

    /* 舵机自定义角度命令：sa000 ~ sa180 */
    if (cmd[0] == 's' && cmd[1] == 'a' &&
        cmd[2] >= '0' && cmd[2] <= '9' &&
        cmd[3] >= '0' && cmd[3] <= '9' &&
        cmd[4] >= '0' && cmd[4] <= '9')
    {
        int angle = (cmd[2] - '0') * 100 +
                    (cmd[3] - '0') * 10 +
                    (cmd[4] - '0');

        if (angle < 0 || angle > 180)
        {
            App_Log("angle out of range\r\n");
            return;
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "servo angle=%d\r\n", angle);
        App_Log(msg);

        Servo_SetAngle(1, angle);
        return;
    }

    /* 舵机开盖命令：op000 */
    if (strncmp(cmd, "op000", 5) == 0)
    {
        App_Log("servo open\r\n");
        Servo_Open();
        return;
    }

    /* 舵机关盖命令：cl000 */
    if (strncmp(cmd, "cl000", 5) == 0)
    {
        App_Log("servo close\r\n");
        Servo_Close();
        return;
    }

    /* 电机移动命令：fs1p5 / bs2p3 */
    char dir = cmd[0];
    char s_flag = cmd[1];
    char sec_char = cmd[2];
    char p_flag = cmd[3];
    char dec_char = cmd[4];

    if ((dir != 'f' && dir != 'b') ||
        s_flag != 's' ||
        p_flag != 'p' ||
        sec_char < '0' || sec_char > '9' ||
        dec_char < '0' || dec_char > '9')
    {
        App_Log("invalid format\r\n");
        return;
    }

    uint32_t move_time_ms = (uint32_t)(sec_char - '0') * 1000U
                          + (uint32_t)(dec_char - '0') * 100U;

    char msg[64];
    snprintf(msg, sizeof(msg), "cmd=%s, time=%u ms\r\n", cmd, move_time_ms);
    App_Log(msg);

    if (dir == 'f')
    {
        App_Log("forward\r\n");
        Motor_Run(80, 80);   // 如果你现在的 Motor_Run 是方向版
        HAL_Delay(move_time_ms);
        Motor_Stop();
        App_Log("stop\r\n");
    }
    else if (dir == 'b')
    {
        App_Log("backward\r\n");
        Motor_Run(-80, -80);
        HAL_Delay(move_time_ms);
        Motor_Stop();
        App_Log("stop\r\n");
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  HCSR04_Init();
  Servo_Init();
  App_Log("boot\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // Motor_RunForward(80);
    // Motor_RunForward(20);
    // Motor_Left(40);
    
    static uint32_t last_measure_tick = 0U;
    static uint8_t object_present = 0;

    static char bt_buf[8];
    static uint8_t bt_idx = 0;

    uint8_t rx_char;

    /* 1. 非阻塞接收蓝牙数据 */
    if (HAL_UART_Receive(&huart1, &rx_char, 1, 10) == HAL_OK)
{
    /* 忽略回车换行 */
    if (rx_char == '\r' || rx_char == '\n')
    {
        bt_idx = 0;
    }
    else
    {
        if (bt_idx < sizeof(bt_buf) - 1)
        {
            bt_buf[bt_idx++] = (char)rx_char;

            /* 收满5个字符就执行，例如 fs1p5 / bs2p3 */
            if (bt_idx == 5)
            {
                bt_buf[bt_idx] = '\0';
                BT_ProcessCommand(bt_buf);
                bt_idx = 0;
            }
        }
        else
        {
            bt_idx = 0;
            App_Log("cmd too long\r\n");
        }
    }
}
    
    if ((HAL_GetTick() - last_measure_tick) >= 200U)
    {
        last_measure_tick = HAL_GetTick();

        if (HCSR04_Measure())
        {
            float distance = HCSR04_GetDistanceCm();

            if (distance > 0 && distance < 20.0f)
            {
                if (object_present == 0)
                {
                    object_present = 1;

                    char msg[64];
                    snprintf(msg, sizeof(msg), "object detected, distance = %.2f cm\r\n", distance);
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

                    for (int j = 0; j < 2; j++)
                    {
                        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                        HAL_Delay(200);
                        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                        HAL_Delay(200);
                    }
                }
            }
            else
            {
                object_present = 0;
            }
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
