/**
  ******************************************************************************
  * @file    ball_balance_pid.c
  * @brief   钢球平衡PID控制算法实现
  ******************************************************************************
  */

#include "ball_balance_pid.h"
#include "Emm_V5.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>  // 用于 abs

// 限幅宏
#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
 * @brief  初始化PID控制器
 */
void PID_Init(PID_Controller_t *pid)
{
    // 设置PID参数
    pid->Kp = PID_KP;
    pid->Ki = PID_KI;
    pid->Kd = PID_KD;
    pid->Kdd = PID_KDD;  // 二阶微分

    // 清零状态
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->last_velocity = 0.0f;  // 初始化上次速度
    pid->error_sum = 0.0f;
    pid->output_pulses = 0;

    // 位置初始化为0（假设上电时滑槽已调平）
    pid->current_position = 0;

    // 稳定状态初始化
    pid->stable_count = 0;
    pid->is_stable = false;
    pid->last_stable_time = 0;

    // 软启动计数器初始化
    pid->soft_start_counter = 0;

    // 默认使能
    pid->enabled = true;
    pid->last_update = HAL_GetTick();
}

/**
 * @brief  PID计算（带软件限位保护和稳定判断）
 */
int32_t PID_Compute(PID_Controller_t *pid, float ball_error)
{
    // 如果未使能，返回0
    if(!pid->enabled)
    {
        return 0;
    }

    // 更新误差
    pid->error = ball_error;

    // ========== 暂时禁用稳定判断，避免静止时不调节 ==========
    // 注释掉稳定判断，确保持续控制
    pid->stable_count = 0;
    pid->is_stable = false;

    // ========== 死区判断（只在误差很小时不控制）==========
    if(fabsf(ball_error) < DEADBAND_ERROR)
    {
        pid->error_sum = 0.0f;  // 清除积分
        return 0;
    }

    // 计算误差变化率（微分）
    // 使用MaixCAM传来的速度（更准确）
    extern float ball_velocity;
    float delta_error = ball_velocity;  // 一阶微分（速度）

    // 计算加速度（二阶微分）⭐⭐⭐
    float acceleration = ball_velocity - pid->last_velocity;
    pid->last_velocity = ball_velocity;  // 保存当前速度

    // 积分计算（带限幅）
    pid->error_sum += pid->error;
    pid->error_sum = CONSTRAIN(pid->error_sum, -MAX_INTEGRAL, MAX_INTEGRAL);

#if CASCADE_PID_MODE
    // ========== 串级PID控制（修正版）⭐⭐⭐ ==========

    // 环1：位置环 → 输出期望速度
    // 负号：球在右侧(error>0)时，期望向左运动(负速度)
    float desired_velocity = -PID_KP_POSITION * pid->error;       // 期望速度
    float velocity_i = -PID_KI_POSITION * pid->error_sum;         // I项修正
    desired_velocity += velocity_i;

    // 环2：速度环 → 纠正速度误差 ⭐⭐⭐ 关键修正！
    // speed_error > 0 → 需要加速向右 → 向右倾斜(正输出)
    // speed_error < 0 → 需要加速向左 → 向左倾斜(负输出)
    float speed_error = desired_velocity - ball_velocity;
    float speed_output = PID_KP_SPEED * speed_error;

    // 速度环微分项（基于加速度，增加阻尼）
    float speed_d = PID_KD_SPEED * acceleration;

    float output = speed_output + speed_d;

    // 串级PID的符号逻辑已经正确，不需要反转

    // 调试信息（串级PID专用）
    float p_term = desired_velocity;  // 显示期望速度
    float i_term = velocity_i;        // 显示积分项
    float d_term = speed_error;       // 显示速度误差
    float dd_term = speed_output;     // 显示速度环输出

#else
    // ========== 传统PID控制 ==========

#if SOFT_START_ENABLE
    // ========== 软启动策略 ==========
    // 前N帧使用降低的Kp和更小的步长，避免初始大幅振荡
    float current_kp = pid->Kp;
    int32_t current_max_pulses = MAX_PULSES;

    if(pid->soft_start_counter < SOFT_START_FRAMES)
    {
        // 软启动阶段
        current_kp = pid->Kp * SOFT_START_KP_RATIO;  // Kp降低到50%
        current_max_pulses = SOFT_START_MAX_PULSES;   // 最大8脉冲
        pid->soft_start_counter++;
    }
#else
    float current_kp = pid->Kp;
    int32_t current_max_pulses = MAX_PULSES;
#endif

    // PID + 前馈计算 ⭐⭐⭐
    float p_term = current_kp * pid->error;        // 比例项
    float i_term = pid->Ki * pid->error_sum;       // 积分项
    float d_term = -pid->Kd * delta_error;         // 微分项（速度阻尼）⭐ 负号
    float dd_term = pid->Kdd * acceleration;       // 二阶微分（加速度）

#if FEEDFORWARD_ENABLE
    // 前馈控制（预测补偿，消除滞后）⭐⭐⭐
    float feedforward = FEEDFORWARD_POSITION * pid->error -
                        FEEDFORWARD_VELOCITY * ball_velocity;  // ⭐ 负号
#else
    float feedforward = 0.0f;
#endif

    float output = p_term + i_term + d_term + dd_term + feedforward;

#endif  // CASCADE_PID_MODE

    // 转换为脉冲数
    int32_t pulses = (int32_t)output;

#if CASCADE_PID_MODE
    // 串级PID使用固定限幅
    int32_t current_max_pulses = MAX_PULSES;
#endif

    // 限幅（使用软启动的限制或固定限制）
    pulses = CONSTRAIN(pulses, -current_max_pulses, current_max_pulses);

#if VELOCITY_DECAY_ENABLE
    // ========== 智能速度衰减 ==========
    // 当球速度很小时，减小输出，避免过度扰动
    float abs_velocity = fabsf(ball_velocity);
    if(abs_velocity < VELOCITY_THRESHOLD)
    {
        // 速度越小，衰减越多
        float decay = abs_velocity / VELOCITY_THRESHOLD;  // 0-1
        decay = decay * (1.0f - VELOCITY_DECAY_FACTOR) + VELOCITY_DECAY_FACTOR;  // 0.5-1.0
        pulses = (int32_t)(pulses * decay);
    }
#endif

#if ENABLE_SOFT_LIMIT
    // ========== 软件限位保护 ==========
    // 检查执行此次移动后是否会超过限位
    int32_t new_position = pid->current_position + pulses;

    if(new_position > MAX_POSITION_PULSES)
    {
        // 超过正向限位，限制脉冲数
        pulses = MAX_POSITION_PULSES - pid->current_position;
        if(pulses < 0) pulses = 0;  // 已经超限，禁止移动
    }
    else if(new_position < -MAX_POSITION_PULSES)
    {
        // 超过反向限位，限制脉冲数
        pulses = -MAX_POSITION_PULSES - pid->current_position;
        if(pulses > 0) pulses = 0;  // 已经超限，禁止移动
    }
#endif

    // 死区判断：脉冲数太小不发送
    if(abs(pulses) < DEADBAND_PULSES)
    {
        pulses = 0;
    }

    // ⭐ 调试输出已禁用，使用main.c中的VOFA+波形输出
    /*
    extern UART_HandleTypeDef huart3;
    char debug[120];

#if CASCADE_PID_MODE
    // 串级PID模式：显示期望速度、速度误差、输出
    // 手动转换浮点数（STM32的sprintf可能不支持%f）
    int err_i = (int)(pid->error * 100);
    int vel_i = (int)(delta_error * 100);
    int desV_i = (int)(p_term * 100);
    int spdErr_i = (int)(d_term * 100);
    int len = sprintf(debug, "[CASCADE] err=%d.%02d, vel=%d.%02d, desV=%d.%02d, spdErr=%d.%02d, out=%ld\r\n",
                     err_i/100, abs(err_i%100), vel_i/100, abs(vel_i%100),
                     desV_i/100, abs(desV_i%100), spdErr_i/100, abs(spdErr_i%100), pulses);
#else
    // 传统PID模式
    int err_i = (int)(pid->error * 100);
    int vel_i = (int)(delta_error * 100);
    int p_i = (int)(p_term * 100);
    int d_i = (int)(d_term * 100);
    int i_i = (int)(i_term * 100);
    int len = sprintf(debug, "[PID] err=%d.%02d, vel=%d.%02d, P=%d.%02d, D=%d.%02d, I=%d.%02d, out=%ld\r\n",
                     err_i/100, abs(err_i%100), vel_i/100, abs(vel_i%100),
                     p_i/100, abs(p_i%100), d_i/100, abs(d_i%100),
                     i_i/100, abs(i_i%100), pulses);
#endif

    HAL_UART_Transmit(&huart3, (uint8_t*)debug, len, 100);
    */

    // 保存当前误差供下次使用
    pid->last_error = pid->error;

    // 保存输出
    pid->output_pulses = pulses;

    return pulses;
}

/**
 * @brief  重置PID状态
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->last_velocity = 0.0f;  // 重置上次速度
    pid->error_sum = 0.0f;
    pid->output_pulses = 0;

    // 重置软启动计数器
    pid->soft_start_counter = 0;
}

/**
 * @brief  使能/禁用PID控制
 */
void PID_SetEnable(PID_Controller_t *pid, bool enable)
{
    pid->enabled = enable;

    // 禁用时重置状态
    if(!enable)
    {
        PID_Reset(pid);
    }
}

/**
 * @brief  设置PID参数
 */
void PID_SetParams(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

/**
 * @brief  执行电机控制（标准位置模式）
 * @param  pulses: 脉冲数（正=向右倾斜，负=向左倾斜）
 * @retval 0=成功，-1=失败
 */
int PID_ExecuteMotorControl(int32_t pulses)
{
    if(pulses == 0)
    {
        return 0;  // 不需要移动
    }

    // 方向映射：正数→dir=0，负数→dir=1（根据实际电机安装方向）
    uint8_t dir = (pulses > 0) ? 0 : 1;  // ⭐ 改回原来的映射
    uint32_t abs_pulses = (uint32_t)abs(pulses);

    // 使用标准位置模式（相对位置）
    Emm_V5_Pos_Control(MOTOR_ADDR, dir, MOTOR_SPEED, MOTOR_ACC, abs_pulses, 0, 0);

    return 0;
}

/**
 * @brief  更新位置并检查限位（在电机控制后调用）
 * @param  pid: PID控制器指针
 * @param  pulses: 实际发送的脉冲数
 * @retval 0=正常，1=接近限位，2=到达限位
 */
int PID_UpdatePosition(PID_Controller_t *pid, int32_t pulses)
{
    // 更新当前位置
    pid->current_position += pulses;

    // 检查是否接近或到达限位
    int32_t abs_position = abs(pid->current_position);

    if(abs_position >= MAX_POSITION_PULSES)
    {
        // 到达限位
        return 2;
    }
    else if(abs_position >= (MAX_POSITION_PULSES * 0.8))
    {
        // 接近限位（80%）
        return 1;
    }

    return 0;  // 正常
}

/**
 * @brief  手动复位位置（当确认滑槽回到水平时调用）
 * @param  pid: PID控制器指针
 * @retval None
 */
void PID_ResetPosition(PID_Controller_t *pid)
{
    pid->current_position = 0;
}
