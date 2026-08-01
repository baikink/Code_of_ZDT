/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>  // 用于 bool 类型
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

// ========== USART1 接收步进电机数据 ==========
#define CMD_LEN 16  // 步进电机驱动器命令长度

// ========== USART2 接收 MaixCAM 数据 ==========
#define RX_MAIXCAM_SIZE 10  // 接收10字节：包头(1) + 误差float(4) + 速度float(4) + 校验和(1)
#define PACKET_HEADER 0xAA  // 包头
#define PACKET_TAIL 0x55    // 包尾

extern __IO uint8_t rxCmd[CMD_LEN];
extern __IO bool rxFrameFlag;
extern __IO uint8_t rxCount;

extern __IO uint8_t rxMaixcam[RX_MAIXCAM_SIZE];
extern __IO float ball_error;     // 小球位置（相对中心，负=左，正=右；由视觉误差取负得到）
extern __IO bool rxMaixcamFlag;

extern __IO uint32_t maixcam_rx_cnt;    // 诊断：收到的帧次数
extern __IO uint32_t maixcam_good_cnt;  // 诊断：校验通过的帧次数

extern __IO uint32_t usart1_rx_cnt;    // 诊断：电机驱动器响应帧次数
extern __IO uint32_t usart1_good_cnt;  // 诊断：位置解析成功帧次数

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

