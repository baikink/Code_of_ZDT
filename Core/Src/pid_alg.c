/**
  * @file    pid_alg.c
  * @brief   PID 算法（位置式 / 增量式）
  *
  * ========== PID 是什么（大白话） ==========
  *
  * PID 就像一个"自动阀门"：
  *   你要浴缸水温 40°C，现在实际 30°C。
  *   P（比例）：偏差 10°C → 把热水阀门拧大一点
  *   I（积分）：半天还没到位？再拧大一点（消除静差）
  *   D（微分）：水温涨太快？提前拧小一点（防超调）
  *
  * 在本项目里：
  *   环1 PID：球偏了 → 算"摆杆该斜多少"
  *   环2 PID：球太快了 → 算"摆杆该往回压多少"
  *
  * ========== 两种模式 ==========
  *   位置式：out = P×e0 + I×累积误差 + D×(e0−e1)  ← 本项目用这个
  *   增量式：out += P×Δe + I×e0 + D×ΔΔe          ← 适合步进电机（累积输出）
  *
  * ========== 使用方法 ==========
  *   1. pid_init(&pid, 模式, P, I, D)  —— 初始化
  *   2. pid.target = 目标值            —— 每帧更新
  *      pid.now    = 当前值
  *   3. pid_cal(&pid)                  —— 计算
  *   4. pid_limit(&pid, max, min)      —— 限幅
  *   5. 用 pid.out 控制执行器
  */
#include "pid_alg.h"

/**
  * @brief  初始化 PID 控制器
  * @param  pid:  控制器指针
  * @param  mode: PID_POSITION(位置式) 或 PID_DELTA(增量式)
  * @param  p,i,d: 三个系数
  */
void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
    pid->mode = mode;
    pid->p = p;
    pid->i = i;
    pid->d = d;

    /* 把所有状态清零，避免垃圾值 */
    pid->target = 0.0f;
    pid->now    = 0.0f;
    pid->out    = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
}

/**
  * @brief  执行一次 PID 计算
  *         调用前必须先设置 pid->target 和 pid->now
  *         结果存在 pid->out 里
  */
void pid_cal(pid_t *pid)
{
    /* ── 第 1 步：算出当前误差 ── */
    pid->error[0] = pid->target - pid->now;

    /* ── 第 2 步：按模式计算 ── */
    if (pid->mode == PID_DELTA) {
        /* 增量式：
           P项 = Kp × (本次误差 − 上次误差)
           I项 = Ki × 本次误差
           D项 = Kd × (本次误差 − 2×上次误差 + 上上次误差)
           输出 = 上次输出 + P + I + D
           优点：输出是增量，天然防积分饱和 */
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0]
                            - 2.0f * pid->error[1]
                            + pid->error[2]);
        pid->out += pid->pout + pid->iout + pid->dout;
    } else {
        /* 位置式（默认，本项目用这个）：
           P项 = Kp × 本次误差
           I项 = Ki × 累积误差（每次累加）
           D项 = Kd × (本次误差 − 上次误差) = 误差变化率
           输出 = P + I + D
           优点：简单直观，参数含义清楚 */
        pid->pout = pid->p * pid->error[0];
        pid->iout += pid->i * pid->error[0];  /* 积分累加 */
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        pid->out = pid->pout + pid->iout + pid->dout;
    }

    /* ── 第 3 步：把这次误差存起来，下次计算用 ── */
    pid->error[2] = pid->error[1];   /* 再上次 ← 上次 */
    pid->error[1] = pid->error[0];   /* 上次   ← 本次 */
}

/**
  * @brief  输出限幅 —— 保证 pid->out 不超过安全范围
  * @param  max_out: 允许的最大输出
  * @param  min_out: 允许的最小输出（可以为负数）
  *
  * 在本项目中：
  *   位置环限幅 ±15 cm/s
  *   速度环限幅 ±300 脉冲
  */
void pid_limit(pid_t *pid, float max_out, float min_out)
{
    if (pid->out > max_out) {
        pid->out = max_out;
    }
    if (pid->out < min_out) {
        pid->out = min_out;
    }
}
