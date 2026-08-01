/**
  * @file    ball_balance.c
  * @brief   钢球平衡双环控制 —— 实现
  *
  * ========== 完整控制流程（每次视觉帧来，跑一遍） ==========
  *
  * 第1步：环1 位置环 —— 球偏了，算一个脉冲把它推回去
  *   - 输入：目标位置（你想球停在哪）vs 球当前实际位置（视觉测的）
  *   - 误差 = 目标 − 实际
  *   - PID 算出"驱动脉冲"：误差大 → 脉冲大，摆杆使劲倾斜
  *
  * 第2步：环2 速度环 —— 球在动，算一个反向脉冲给它刹车
  *   - 输入：目标速度（= 0，希望球静止）vs 球当前实际速度（视觉测的）
  *   - 误差 = 0 − 实际速度
  *   - PID 算出"阻尼脉冲"：球越快 → 反向力越大 → 防止冲过头
  *
  * 第3步：两个脉冲相加，限幅 +300，发给步进电机
  *   - 正脉冲 = 摆杆上升(CW)
  *   - 负脉冲 = 摆杆下降(CCW)
  *
  * ========== 举个例子（球在左边 +5cm，正在继续往左滚） ==========
  *   ball_error = +5（球在左边，正值=左），ball_velocity = +3（往左滚）
  *   环1 误差 = 0 - (+5) = -5 → 算出负脉冲 → 摆杆下降 → 球往右推
  *   环2 误差 = 0 - (+3) = -3 → 算出负脉冲 → 和环1同向，加强推力
  *   总脉冲 < 0 → CCW → 摆杆下降 → 球往右 → 减速 → 折返 → 回到中心
  *
  * ========== 关于速度测量 ==========
  * 球的速度由 MaixCAM 端计算（相邻两帧位置差 / 时间间隔），
  * 通过串口发给 STM32，存入 ball_velocity 变量。
  * STM32 端不需要自己算速度，直接用这个值即可。
  */
#include "ball_balance.h"
#include "pid_alg.h"
#include "Emm_V5.h"
#include "usart.h"

/* ── 两个 PID 控制器（static 表示只在本文件内使用）── */
static pid_t pos_pid;   /* 环1：位置环 */
static pid_t spd_pid;   /* 环2：速度环 */

/* ── 调试变量（供 main.c 的 VOFA 发送读取）── */
float  dbg_target_cm     = 0.0f;  /* 环1 目标位置 */
float  dbg_speed_out     = 0.0f;  /* 环2 速度环输出 */
int32_t dbg_total_pulses = 0;     /* 总输出脉冲 */

/**
  * @brief  上电时调用一次：初始化两个 PID + 配置电机
  *         在 main.c 的 while(1) 之前调用
  */
void ball_balance_init(void)
{
    /* 环1 初始化：位置 PID，参数从 ball_balance.h 的 POS_KP/KI/KD 来 */
    pid_init(&pos_pid, PID_POSITION, POS_KP, POS_KI, POS_KD);

    /* 环2 初始化：速度 PID，目标速度永远是 0 */
    pid_init(&spd_pid, PID_POSITION, SPD_KP, SPD_KI, SPD_KD);
    spd_pid.target = 0.0f;   /* 环2 的目标：球速度 = 0（静止） */

    /* 告诉电机：准备用"快速位置模式"，运动参数为 1500RPM / 加速度100
       raF=2 的含义："相对当前电机实时位置进行相对位置运动"
       即每次发 QPos_Control(pulse) 时，pulse 是从"此刻位置"的相对偏移量
       正=CW=上升，负=CCW=下降 */
    Emm_V5_Set_QPos_Params(BALL_MOTOR_ADDR,
                           BALL_MOTOR_VEL,
                           BALL_MOTOR_ACC,
                           2,   /* raF=2：相对当前实时位置 */
                           0);  /* snF=0：不启用多机同步 */
}

/**
  * @brief  双环控制 —— 每次收到一帧视觉数据时调用（约 60fps / 每 17ms）
  * @param  target_cm: 目标球位置(cm)，0 = 摆杆中心点
  *
  * ball_error 和 ball_velocity 由 USART2 中断自动更新，
  * 这里直接拿来用就行，不需要自己读串口。
  */
void ball_balance_control(float target_cm)
{
    float pulse_a = 0.0f;   /* 环1 算出的驱动脉冲 */
    float pulse_b = 0.0f;   /* 环2 算出的阻尼脉冲 */
    float total   = 0.0f;   /* 最终发给电机的脉冲 */
    int32_t pulses = 0;     /* 取整后的脉冲数 */

    /* ═══════════════════════════════════════════════
     * 环1：位置环 —— "球偏了，推回去"
     * ═══════════════════════════════════════════════ */
    pos_pid.target = target_cm;          /* 你希望球停在哪个位置 */
    pos_pid.now    = ball_error;         /* 球现在实际在哪（视觉测的）*/
    pid_cal(&pos_pid);                   /* PID 计算，结果存在 pos_pid.out */
    pid_limit(&pos_pid,                  /* 输出限幅，防止脉冲过大 */
              (float)MAX_PULSE_CW,       /*   上限 +300 */
              (float)MAX_PULSE_CCW);     /*   下限 -300 */
    pulse_a = pos_pid.out;               /* 取出环1 的结果 */

    /* ═══════════════════════════════════════════════
     * 环2：速度环 —— "球在动，刹住它"
     * ═══════════════════════════════════════════════ */
    spd_pid.target = 0.0f;               /* 目标速度：0 = 停住 */
    spd_pid.now    = ball_velocity;      /* 球现在实际速度（视觉测的）*/
    pid_cal(&spd_pid);                   /* PID 计算，结果存在 spd_pid.out */
    pid_limit(&spd_pid,                  /* 输出限幅 */
              (float)MAX_PULSE_CW,
              (float)MAX_PULSE_CCW);
    pulse_b = spd_pid.out;               /* 取出环2 的结果 */

    /* ═══════════════════════════════════════════════
     * 合起来：驱动 + 阻尼，再限一次幅保证安全
     * ═══════════════════════════════════════════════ */
    total = pulse_a + pulse_b;

    /* 硬限幅：绝不超 +300 */
    if (total > (float)MAX_PULSE_CW) {
        total = (float)MAX_PULSE_CW;
    }
    if (total < (float)MAX_PULSE_CCW) {
        total = (float)MAX_PULSE_CCW;
    }

    /* 转成整数发给电机 */
    pulses = (int32_t)total;

    /* ── 更新调试变量（VOFA 实时显示）── */
    dbg_target_cm     = target_cm;       /* 环1 目标位置 */
    dbg_speed_out     = spd_pid.out;     /* 环2 速度环输出 */
    dbg_total_pulses  = pulses;          /* 总输出脉冲 */

    if (pulses != 0) {
        /* 发给步进电机：正=CW(上升)，负=CCW(下降) */
        Emm_V5_QPos_Control(BALL_MOTOR_ADDR, pulses);
    }
}
