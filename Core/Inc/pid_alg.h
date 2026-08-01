/**
  * @file    pid_alg.h
  * @brief   PID 算法核心（位置式/增量式），从参考代码提取精简
  */
#ifndef __PID_ALG_H
#define __PID_ALG_H

#include <stdint.h>

/* PID 模式 */
enum pid_mode {
    PID_POSITION = 0,   // 位置式 PID
    PID_DELTA,          // 增量式 PID
};

/* PID 控制器结构体 */
typedef struct {
    float target;       // 目标值
    float now;          // 当前反馈值
    float error[3];     // [0]=本次, [1]=上次, [2]=上上次
    float p, i, d;      // 系数
    float pout, iout, dout; // 各分量
    float out;           // 总输出
    uint32_t mode;       // 模式
} pid_t;

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);
void pid_cal(pid_t *pid);
void pid_limit(pid_t *pid, float max_out, float min_out);

#endif
