#ifndef __VOFA_USAGE_H
#define __VOFA_USAGE_H

#include <stdint.h>

/**
  * @brief  VOFA+ 通道初始化（STM32 上 USART3 已在 MX_USART3_UART_Init 配置，
  *         此函数保留仅为兼容原接口）。
  */
void Vofa_usage_prepare(void);

/**
  * @brief  发送单个字符到 VOFA+（USART3）。
  */
void Vofa_usage_SendByte(uint8_t Byte);

/**
  * @brief  发送一组整数到 VOFA+（FireWater：逗号分隔，\n 结尾）。
  * @param  Arrays: 待发送的 int32 数组
  * @param  num:    数组元素个数
  */
void Vofa_usage_SendString(int32_t *Arrays, uint8_t num);

#endif /* __VOFA_USAGE_H */
