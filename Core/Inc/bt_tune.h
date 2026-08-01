#ifndef __BT_TUNE_H__
#define __BT_TUNE_H__

#include "main.h"

/* ── 蓝牙调参模块（UART4: PC10=TX / PC11=RX）──
 *
 * App 端滑杆映射（[s,编号,值] 或 [slider,编号,值]）：
 *   1 → pidY.p       位置环 P
 *   2 → pidY.i       位置环 I
 *   3 → pidY.d       位置环 D
 *   4 → pidY_Speed.p 速度环 P
 *   5 → pidY_Speed.i 速度环 I
 *   6 → pidY_Speed.d 速度环 D
 *
 * App 端按键（[k,标识,d]）：
 *   rst → 清除 pidY 和 pidY_Speed 的积分
 *
 * 短格式均支持：[s,1,2.50] 等效 [slider,1,2.50]
 */

#define BT_BUF_SIZE  64   /* 单包最大字节数（含 '\0'） */

void BT_Tune_Init(void);          /* main 初始化中调用，启动 UART4 中断接收 */
void BT_Tune_ByteReceived(void);  /* 在 HAL_UART_RxCpltCallback 中调用     */
void BT_Tune_Process(void);       /* 在主循环中调用，解析并应用参数         */
void BT_Tune_SendStatus(void);    /* 可选：回传当前 PID 参数到 App 显示区   */

#endif /* __BT_TUNE_H__ */
