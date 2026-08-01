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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Emm_V5.h"
#include "UART_vofa_usage.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ========== 步进电机通信参数 ==========
#define MOTOR_ADDR  1      // 电机地址
#define MOTOR_SPEED 1500   // RPM
#define MOTOR_ACC   100    // 加速度档位

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 钢球位置误差/速度（由 USART2 中断从 MaixCAM 数据解析）
float ball_velocity = 0.0f;        // 钢球速度（cm/s）

// ── PID 实例（声明在 pid.h）──

// ── 10ms 定时标志（SysTick 派生，100Hz）──
volatile uint32_t g_tick_10ms = 0;
volatile bool g_flag_10ms = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // ========== 启动 UART1 DMA 接收（步进电机） ==========
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)rxCmd, CMD_LEN);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  // 使能 UART 空闲中断

  // ========== 启动 UART2 DMA 接收（MaixCAM 视觉） ==========
  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxMaixcam, RX_MAIXCAM_SIZE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);  // 使能 UART 空闲中断

  // ========== VOFA+ 通道准备（USART3） ==========
  Vofa_usage_prepare();

  // ========== 步进电机初始化 ==========
  HAL_Delay(500);  // 等待串口和电机驱动器就绪
/*
  // 使能电机（只通电锁轴，不会转动）
  Emm_V5_En_Control(MOTOR_ADDR, 1, 0);
  HAL_Delay(100);
*/
  // 等待驱动器上电自动回零完成（0点已在张大头上位机设置好）
  HAL_Delay(3000);
  HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t last_vofa_time = HAL_GetTick();  // 上次读取+发送时间

  pid_init(&pidY, POSITION_PID, 1.20f, 0.0f, 0.0f, 0.9f);//2.50f, 0.0f, 0.70f, 0.0f
  pid_init(&pidY_Speed, POSITION_PID, 1.1f, 0.0f, 0.0f, 0.0f);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 if(g_flag_10ms)
    {
        g_flag_10ms = false;    // 清标志
        pid_control__26Y();
    }

    // ========== 视觉数据：每帧更新 ==========
    if(rxMaixcamFlag)
    {
        rxMaixcamFlag = false;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }


    // ========== 每10ms：发送 VOFA+ 数据（100Hz） ==========
    if(HAL_GetTick() - last_vofa_time >= 10)
    {
      last_vofa_time = HAL_GetTick();

      // ===== VOFA+ 发送：5通道 =====
      // 所有通道均 ×100 发送，VOFA+ 中 ÷100 后为真实值。
      int32_t vofa_data[5];
      vofa_data[0] = (int32_t)(motor_get_command_angle() * 100.0f);  // ch0: 开环目标角度(°)
      vofa_data[1] = (int32_t)(Y * 100.0f);                          // ch1: 小球位置(cm)
      vofa_data[2] = (int32_t)(ball_velocity * 100.0f);              // ch2: MaixCAM 计算的小球速度(cm/s)
      vofa_data[3] = (int32_t)(pidY.out * 100.0f);                   // ch3: 位置环输出(°)
      vofa_data[4] = (int32_t)(pidY_Speed.out * 100.0f);             // ch4: 速度环输出
      Vofa_usage_SendString(vofa_data, 5);
    }

    // ========== 处理 MaixCAM 视觉数据 ==========
    if(rxMaixcamFlag == true)
    {
      rxMaixcamFlag = false;

      // LED指示：收到一帧视觉数据
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
