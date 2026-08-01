/**
  ******************************************************************************
  * @file    UART_vofa_usage.c
  * @brief   VOFA+ 上位机数据发送模块（STM32 HAL 移植版）
  *
  *          原代码来自 TI MSPM0 平台（ti_msp_dl_config.h / DL_UART_Main_*），
  *          已移植为 STM32F407 + HAL 库，通过 USART3 发送到电脑。
  *
  *          输出协议：VOFA+ FireWater（ASCII 文本）
  *          每帧格式："整数,整数,...,整数\n"（逗号分隔，换行结尾）
  *
  * @note    需要先在 main.c 中调用 MX_USART3_UART_Init() 初始化 USART3。
  ******************************************************************************
  */
#include "UART_vofa_usage.h"
#include "usart.h"   // huart3（USART3 = 到电脑的串口）

/**
  * @brief  VOFA+ 通道初始化。
  *         本工程 USART3 已由 MX_USART3_UART_Init() 完成 GPIO 与波特率配置，
  *         纯发送不需要开中断，此函数保留仅为兼容原接口约定。
  */
void Vofa_usage_prepare(void)
{
    // USART3 已在 MX_USART3_UART_Init() 中初始化，此处无需额外配置
}

/**
  * @brief  发送单个字符到 VOFA+（走 USART3）。
  * @param  Byte: 要发送的字符
  */
void Vofa_usage_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart3, &Byte, 1, 100);
}

/**
  * @brief  发送一组整数到 VOFA+（FireWater 协议：逗号分隔，\n 结尾）。
  * @param  Arrays: 待发送的 int32 数组
  * @param  num:    数组元素个数
  * @note   小数请先放大为整数（如 ×100）再发送，
  *         在 VOFA+ 里看到的数值 ÷100 即为真实值。
  */
void Vofa_usage_SendString(int32_t *Arrays, uint8_t num)
{
    for(uint8_t i = 0; i < num; i++)
    {
        int32_t val = Arrays[i];

        // 负数：先输出负号，再处理绝对值
        if(val < 0)
        {
            Vofa_usage_SendByte('-');
            val = -val;
        }

        // 逐位拆出数字（自然长度，无前导零）
        char digits[12];
        uint8_t n = 0;
        if(val == 0)
        {
            digits[n++] = '0';
        }
        while(val > 0)
        {
            digits[n++] = (char)('0' + val % 10);
            val /= 10;
        }
        while(n > 0)
        {
            Vofa_usage_SendByte((uint8_t)digits[--n]);
        }

        // 分隔符：非最后一个元素用逗号，最后一个用换行
        if(i != num - 1)
        {
            Vofa_usage_SendByte(',');
        }
        else
        {
            Vofa_usage_SendByte('\n');
        }
    }
}
