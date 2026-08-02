/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Emm_V5.h"
#include <string.h>  // 用于 memcpy
#include <math.h>    // 用于 fabsf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern volatile uint32_t g_tick_10ms;
extern volatile bool g_flag_10ms;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  static uint32_t tick_cnt = 0;
  tick_cnt++;
  if(tick_cnt >= 20) {
    tick_cnt = 0;
    g_tick_10ms++;
    g_flag_10ms = true;
  }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  // 检测到 UART 空闲中断（接收完一帧数据）
  if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);                       // 清除空闲中断标志
    HAL_UART_DMAStop(&huart1);                                // 停止 DMA 接收
    usart1_rx_cnt++;                                          // 诊断：收到驱动器响应(无论对错)
    rxCount = CMD_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  // 计算接收到的数据长度
    rxFrameFlag = true;                                       // 设置接收完成标志

    // 解析电机位置数据（如果是位置查询的响应）
    // 响应格式：[帧头][地址][方向][位置高位][位置次高位][位置次低位][位置低位][校验和]
    if(rxCount >= 7 && rxCmd[0] == 0x01)  // 帧头0x01，地址匹配
    {
      usart1_good_cnt++;  // 诊断：位置解析成功
      // 开环控制不使用编码器位置；此处只保留驱动器响应的诊断计数。
    }

    HAL_UART_Receive_DMA(&huart1, (uint8_t*)rxCmd, CMD_LEN); // 重新开启 DMA 接收
  }

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  // 检测到 UART 空闲中断（接收完一帧数据）
  if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);  // 清除空闲中断标志
    HAL_UART_DMAStop(&huart2);           // 停止 DMA 接收
    maixcam_rx_cnt++;                    // 诊断：收到一帧(无论对错)

    // 数据包校验：[0xAA][float误差4][float速度4][校验和]
    // 共10字节
    if(rxMaixcam[0] == PACKET_HEADER)
    {
      // 计算校验和（对9个字节求和）
      uint8_t checksum = 0;
      for(int i = 0; i < 9; i++)  // 帧头 + 2个float
      {
        checksum += rxMaixcam[i];
      }
      checksum &= 0xFF;

      // 校验和正确
      if(checksum == rxMaixcam[9])
      {
        maixcam_good_cnt++;  // 诊断：校验通过
        // 解析误差 float 数据（小端格式）
        // 视觉端发送的是 error = 0 - x_cm（取反），这里取负还原为小球位置：
        // 负值=球在中心左侧，正值=球在右侧（与 MaixCAM 屏幕显示一致）⭐
        float new_error;
        memcpy((void*)&new_error, (void*)&rxMaixcam[1], sizeof(float));
        new_error = -new_error;  // 取负：还原为位置

        // 解析速度 float 数据（小端格式）
        float new_velocity;
        memcpy((void*)&new_velocity, (void*)&rxMaixcam[5], sizeof(float));

        // 异常值检测：防止视觉突变
        if(fabsf(new_error) < 15.0f && fabsf(new_velocity) < 50.0f)  // 限制范围
        {
          // 突变检测：与上次误差差距不超过10cm（放宽，避免数据冻结）⭐
          if(fabsf(new_error - ball_error) < 10.0f || ball_error == 0.0f)
          {
            extern volatile float ball_velocity;
            ball_error = new_error;
            ball_velocity = -new_velocity;  // MaixCAM 发送 error 的导数，取负后与 ball_error 坐标同向
            rxMaixcamFlag = true;  // 设置接收完成标志
          }
          else
          {
            // ⭐ 突变过大，但仍设置标志（防止完全卡死）
            // 使用限幅后的数据
            extern volatile float ball_velocity;
            if(new_error > ball_error + 10.0f) {
              ball_error = ball_error + 10.0f;
            } else if(new_error < ball_error - 10.0f) {
              ball_error = ball_error - 10.0f;
            } else {
              ball_error = new_error;
            }
            ball_velocity = -new_velocity;  // 与正常数据分支保持同一坐标方向
            rxMaixcamFlag = true;
          }
        }
        // 否则丢弃此数据包（超出合理范围）
      }
      // 如果校验失败，不设置标志，丢弃此包
    }

    // 重新开启 DMA 接收
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxMaixcam, RX_MAIXCAM_SIZE);
  }

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
