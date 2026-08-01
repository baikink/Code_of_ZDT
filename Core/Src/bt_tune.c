#include "bt_tune.h"
#include "usart.h"
#include "pid.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ── 接收状态 ── */
static uint8_t       bt_rx_byte;                /* 单字节接收目标（HAL_UART_Receive_IT 写入） */
static char          bt_raw[BT_BUF_SIZE];       /* 接收中的包内容（不含 [ ]）                */
static uint8_t       bt_raw_len  = 0;
static char          bt_pkt[BT_BUF_SIZE];       /* 已完整接收的包（供主循环解析）             */
static volatile bool bt_pkt_ready = false;      /* 有包待处理（ISR 写，主循环读）             */

/* ─────────────────────────────────────────────
 * BT_Tune_Init
 * 启动 UART4 单字节中断接收，main() 初始化末尾调用一次
 * ───────────────────────────────────────────── */
void BT_Tune_Init(void)
{
    HAL_UART_Receive_IT(&huart4, &bt_rx_byte, 1);
}

/* ─────────────────────────────────────────────
 * BT_Tune_ByteReceived
 * 在 HAL_UART_RxCpltCallback 中调用（ISR 上下文）
 * 逐字节拼包，收到 ']' 后置 bt_pkt_ready
 * ───────────────────────────────────────────── */
void BT_Tune_ByteReceived(void)
{
    char c = (char)bt_rx_byte;

    if(c == '[') {
        bt_raw_len = 0;                                    /* 包头，重置缓冲 */
    } else if(c == ']') {
        if(bt_raw_len > 0 && bt_raw_len < BT_BUF_SIZE) {
            bt_raw[bt_raw_len] = '\0';
            memcpy(bt_pkt, bt_raw, bt_raw_len + 1);
            bt_pkt_ready = true;                           /* 通知主循环 */
        }
        bt_raw_len = 0;
    } else {
        if(bt_raw_len < BT_BUF_SIZE - 1) {
            bt_raw[bt_raw_len++] = c;
        }
    }

    HAL_UART_Receive_IT(&huart4, &bt_rx_byte, 1);         /* 继续接收下一字节 */
}

/* ─────────────────────────────────────────────
 * BT_Tune_Process
 * 在主循环中调用，解析完整包并写入 PID 参数
 * 支持短格式 [s/slider, 编号, 值] 和 [k/key, 标识, d/u]
 * ───────────────────────────────────────────── */
void BT_Tune_Process(void)
{
    if(!bt_pkt_ready) return;
    bt_pkt_ready = false;

    /* strtok 会修改字符串，先拷贝一份 */
    char tmp[BT_BUF_SIZE];
    strncpy(tmp, bt_pkt, BT_BUF_SIZE - 1);
    tmp[BT_BUF_SIZE - 1] = '\0';

    char *cmd  = strtok(tmp,  ",");
    char *arg1 = strtok(NULL, ",");
    char *arg2 = strtok(NULL, ",");
    if(!cmd || !arg1) return;

    /* ── 滑杆调参 [s,N,val] ── */
    if(strcmp(cmd, "s") == 0 || strcmp(cmd, "slider") == 0) {
        if(!arg2) return;
        int   ch  = atoi(arg1);
        float val = (float)atof(arg2);
        switch(ch) {
            case 1: pidY.p       = val; break;   /* 位置环 P */
            case 2: pidY.i       = val; break;   /* 位置环 I */
            case 3: pidY.d       = val; break;   /* 位置环 D */
            case 4: pidY_Speed.p = val; break;   /* 速度环 P */
            case 5: pidY_Speed.i = val; break;   /* 速度环 I */
            case 6: pidY_Speed.d = val; break;   /* 速度环 D */
            default: break;
        }
    }
    /* ── 按键 [k,rst,d] 清除积分 ── */
    else if(strcmp(cmd, "k") == 0 || strcmp(cmd, "key") == 0) {
        if(strcmp(arg1, "rst") == 0) {
            pidY.iout       = 0.0f;
            pidY_Speed.iout = 0.0f;
        }
    }
}

/* ─────────────────────────────────────────────
 * BT_Tune_SendStatus
 * 把当前 PID 参数以 display 数据包格式回传到 App
 * 在主循环低频调用（例如每 500ms 一次）
 * ───────────────────────────────────────────── */
void BT_Tune_SendStatus(void)
{
    char buf[128];
    int n;

    /* 第一行：位置环参数 */
    n = snprintf(buf, sizeof(buf),
                 "[d,0,0,Pos P:%.2f I:%.3f D:%.2f,18]\r\n",
                 pidY.p, pidY.i, pidY.d);
    if(n > 0) HAL_UART_Transmit(&huart4, (uint8_t*)buf, (uint16_t)n, 20);

    /* 第二行：速度环参数 */
    n = snprintf(buf, sizeof(buf),
                 "[d,0,20,Spd P:%.2f I:%.3f D:%.2f,18]\r\n",
                 pidY_Speed.p, pidY_Speed.i, pidY_Speed.d);
    if(n > 0) HAL_UART_Transmit(&huart4, (uint8_t*)buf, (uint16_t)n, 20);
}
