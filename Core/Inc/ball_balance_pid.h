/**
  ******************************************************************************
  * @file    ball_balance_pid.h
  * @brief   钢球平衡PID控制算法
  * @author  Your Name
  * @date    2026-07-31
  ******************************************************************************
  */

#ifndef __BALL_BALANCE_PID_H
#define __BALL_BALANCE_PID_H

#include "main.h"
#include <stdbool.h>

// ========== PID参数配置 ==========

// ⭐⭐⭐ 串级PID控制模式（参考板球系统）
#define CASCADE_PID_MODE    0        // 暂时禁用串级PID，先用传统PID调试 ⭐

// 环1：位置环参数
#define PID_KP_POSITION     0.5f     // 位置环比例系数（降低，减少期望速度）⭐
#define PID_KI_POSITION     0.03f    // 位置环积分系数（降低）⭐

// 环2：速度环参数
#define PID_KP_SPEED        1.5f     // 速度环比例系数（降低，减弱响应）⭐
#define PID_KD_SPEED        0.3f     // 速度环微分系数（降低）⭐

// 传统PID参数 - 在稳定基础上加入积分
#define PID_KP              0.25f    // 比例系数（略提高）
#define PID_KI              0.04f    // 积分系数（加入，消除稳态误差）⭐
#define PID_KD              3.0f     // 一阶微分（略降低，保持稳定）
#define PID_KDD             0.0f     // 二阶微分禁用

// 前馈控制 - 略降低，配合积分项
#define FEEDFORWARD_ENABLE     1        // 启用前馈
#define FEEDFORWARD_POSITION   0.0f     // 位置前馈系数
#define FEEDFORWARD_VELOCITY   0.4f     // 速度前馈系数（略降低）⭐

// 控制参数
#define DEADBAND_ERROR      0.02f    // 死区：0.02cm（降低，让小误差也调节）⭐
#define DEADBAND_PULSES     0        // 死区：0脉冲（禁用脉冲死区）⭐
#define MAX_PULSES          15       // 单次最大移动脉冲数 ⭐ 提高到15，增强控制力度
#define MAX_INTEGRAL        8.0f     // 积分限幅（提高，让积分有足够作用）⭐

// 软启动策略（大幅缩短）⭐⭐⭐
#define SOFT_START_ENABLE      0        // 禁用软启动 ⭐ 立即全力控制
#define SOFT_START_FRAMES      10       // 软启动帧数（缩短到10帧=0.03秒）
#define SOFT_START_KP_RATIO    0.8f     // 软启动Kp比例（提高到0.8）
#define SOFT_START_MAX_PULSES  8        // 软启动最大8脉冲

// 速度相关衰减（禁用）
#define VELOCITY_DECAY_ENABLE  0        // 禁用速度衰减
#define VELOCITY_THRESHOLD     5.0f
#define VELOCITY_DECAY_FACTOR  0.7f

// 稳定判断参数（暂时禁用）
#define STABLE_ERROR_THRESHOLD  0.3f    // 稳定误差阈值(cm)
#define STABLE_TIME_MS          2000    // 稳定时间(ms)
#define STABLE_COUNT_THRESHOLD  100     // 稳定次数阈值

// ========== 安全限位保护（防止机械损坏）==========
#define MAX_ANGLE_LIMIT     60.0f    // 最大允许角度：±60°
#define MAX_POSITION_PULSES 1066     // 最大位置脉冲数（32细分：60° = 17.78×60 ≈ 1066脉冲）
#define ENABLE_SOFT_LIMIT   0        // 暂时禁用软件限位：1=开启，0=关闭

// 细分设置说明
// 16细分：3200脉冲/圈，1脉冲 = 0.1125°，1° = 8.89脉冲
// 32细分：6400脉冲/圈，1脉冲 = 0.05625°，1° = 17.78脉冲 ← 当前使用

// 电机参数（温和响应）⭐
#define MOTOR_SPEED         1500     // RPM（降低到1500）
#define MOTOR_ACC           100      // 加速度档位（降低到100）
#define MOTOR_ADDR          1        // 电机地址

// 控制周期
#define CONTROL_PERIOD_MS   10       // 控制周期10ms（100Hz）⭐ 参考代码的节奏

// ========== PID控制器结构体 ==========

typedef struct {
    // PID参数
    float Kp;
    float Ki;
    float Kd;
    float Kdd;  // 二阶微分系数（加速度）⭐

    // PID状态
    float error;          // 当前误差
    float last_error;     // 上次误差
    float last_velocity;  // 上次速度（用于计算加速度）⭐
    float error_sum;      // 误差积分

    // 输出
    int32_t output_pulses;  // 输出脉冲数

    // 位置跟踪（软件限位）
    int32_t current_position;  // 当前位置（相对初始位置的累计脉冲）

    // 稳定状态判断（防止反复抖动）
    uint32_t stable_count;     // 连续稳定计数
    bool is_stable;            // 稳定标志
    uint32_t last_stable_time; // 上次稳定时间

    // 软启动计数器
    uint32_t soft_start_counter;  // 软启动帧计数

    // 状态
    bool enabled;         // 使能标志
    uint32_t last_update; // 上次更新时间

} PID_Controller_t;

// ========== 函数声明 ==========

/**
 * @brief  初始化PID控制器
 * @param  pid: PID控制器指针
 * @retval None
 */
void PID_Init(PID_Controller_t *pid);

/**
 * @brief  PID计算
 * @param  pid: PID控制器指针
 * @param  ball_error: 钢球位置误差（cm），正值=右侧，负值=左侧
 * @retval 输出脉冲数（正值=CCW，负值=CW）
 */
int32_t PID_Compute(PID_Controller_t *pid, float ball_error);

/**
 * @brief  重置PID状态
 * @param  pid: PID控制器指针
 * @retval None
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief  使能/禁用PID控制
 * @param  pid: PID控制器指针
 * @param  enable: true=使能，false=禁用
 * @retval None
 */
void PID_SetEnable(PID_Controller_t *pid, bool enable);

/**
 * @brief  设置PID参数
 * @param  pid: PID控制器指针
 * @param  kp, ki, kd: PID系数
 * @retval None
 */
void PID_SetParams(PID_Controller_t *pid, float kp, float ki, float kd);

/**
 * @brief  执行电机控制（位置模式）
 * @param  pulses: 脉冲数（正=CCW，负=CW）
 * @retval 0=成功，-1=失败
 */
int PID_ExecuteMotorControl(int32_t pulses);

/**
 * @brief  更新位置并检查限位
 * @param  pid: PID控制器指针
 * @param  pulses: 实际发送的脉冲数
 * @retval 0=正常，1=接近限位（80%），2=到达限位
 */
int PID_UpdatePosition(PID_Controller_t *pid, int32_t pulses);

/**
 * @brief  手动复位位置（当确认滑槽回到水平时调用）
 * @param  pid: PID控制器指针
 * @retval None
 */
void PID_ResetPosition(PID_Controller_t *pid);

#endif /* __BALL_BALANCE_PID_H */
