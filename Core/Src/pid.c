#include "pid.h"
#include "Emm_V5.h"              /* ball_error、Emm_V5_Pos_Control */

extern volatile float ball_velocity;  /* 视觉小球速度，定义在 main.c，中断写主循环读 */

#define PULSES_PER_DEG  8.89f  /* 3200脉冲/圈，1°=8.89脉冲 */

/* 上一次下发的绝对目标脉冲，仅用于 VOFA 显示和避免重复发同一目标。 */
static int32_t motor_command_pulses = 0;

/* 由 pid.h 的角度限幅换算成脉冲阈值（仅 pid.c 内部使用） */
#define PULSE_LIMIT_MAX  (ANGLE_LIMIT_MAX * PULSES_PER_DEG)            /* 正向限幅脉冲 */
#define PULSE_LIMIT_MIN  (ANGLE_LIMIT_MIN * PULSES_PER_DEG)            /* 反向限幅脉冲 */
#define PULSE_DEAD_MAX   ((ANGLE_LIMIT_MAX - 2.0f) * PULSES_PER_DEG)  /* 正向死区起点（限位前2°） */
#define PULSE_DEAD_MIN   ((ANGLE_LIMIT_MIN + 2.0f) * PULSES_PER_DEG)  /* 反向死区起点（限位前2°） */

#define MAX_DUTY  7200    /* 旧代码需要，PWM最大占空比 */
#define MOTOR_ADDR 1      /* 旧代码需要，电机地址 */
#define MOTOR_POSITION_SPEED_RPM  100u   /* 串口绝对位置命令速度 */


int redXSpeed;
int redYSpeed;

float Y;
float Y_last;

/* 目标坐标与视觉 Y 使用同一坐标系，默认回到画面中心。 */
volatile float ball_target_y = 0.0f;

void ball_target_set(float target_y)
{
	ball_target_y = target_y;
}

float ball_target_get(void)
{
	return ball_target_y;
}

pid_t pidY;
pid_t pidY_Speed;
float pidY_velocity_damping;
float pidY_filtered_velocity = 0.0f;  /* 低通滤波后的视觉速度 (cm/s)，供 VOFA ch5 */

typedef enum {
	PID_SEGMENT_TO_POS_56 = 0,
	PID_SEGMENT_TO_NEG_55,
	PID_SEGMENT_HOLD_NEG_55,
} pid_segment_t;

static pid_segment_t g_pid_segment = PID_SEGMENT_TO_POS_56;
static bool g_pid_segment_initialized = false;

static float pid_absf(float value)
{
	return (value >= 0.0f) ? value : -value;
}

static void pid_apply_segment_profile(float target_y,
	                                  float pos_p, float pos_i, float pos_d,
	                                  float spd_p, float spd_i, float spd_d)
{
	pid_init(&pidY, POSITION_PID, pos_p, pos_i, pos_d, 0.0f);
	pid_init(&pidY_Speed, POSITION_PID, spd_p, spd_i, spd_d, 0.0f);
	ball_target_set(target_y);
}

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d,float f)
{
	pid->pid_mode = mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
	pid->f = f;
	pid->target = 0;
	pid->now = 0;
	pid->out = 0;
	pid->iout = 0;
	pid->error[0] = pid->error[1] = pid->error[2] = 0;
	pid->target_last = 0;
}


void pid_cal(pid_t *pid)
{
	// ���㵱ǰƫ��
	pid->error[0] = pid->target - pid->now;

	// �������
	if(pid->pid_mode == DELTA_PID)  // ����ʽ
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->fout = pid->f * (pid->target - pid->target_last);
		pid->target_last = pid->target;
		pid->out += pid->pout + pid->iout + pid->dout + pid->fout;
	}
	else if(pid->pid_mode == POSITION_PID)  // λ��ʽ
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// ��¼ǰ����ƫ��
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];

//	// ����޷�
//	if(pid->out>=MAX_DUTY)	
//		pid->out=MAX_DUTY;
//	if(pid->out<=0)	
//		pid->out=0;
	
}

void pidout_limit(pid_t *pid)
{
	PIDOUT_CLAMP(pid, 0, MAX_DUTY);
}

/* 带条件积分与输出限幅的位置式 PID（串级环专用）
 *
 * 与 PIDOUT_CLAMP 的反算抗饱和相比，本函数从不把比例项的溢出量写进 iout：
 *   1. 先试算积分增量，只有「输出未饱和」或「误差方向会让积分退出饱和」时才采纳；
 *   2. iout 独立限幅到 ±i_limit（i_limit=0 即彻底禁用积分，等价纯 P/PD）；
 *   3. 最后只截断 out，iout 保持干净。
 *
 * 这样比例项可以随意撞限幅，积分状态不会被污染，不存在 Ki=0 时留下永久偏置的问题。
 */
static void pid_cal_clamped(pid_t *pid, float lo, float hi, float i_limit)
{
	pid->error[0] = pid->target - pid->now;

	pid->pout = pid->p * pid->error[0];
	pid->dout = pid->d * (pid->error[0] - pid->error[1]);

	/* 条件积分：先看不含新增量的输出是否已经饱和到同一侧。 */
	if(i_limit > 0.0f) {
		float out_wo_new_i = pid->pout + pid->iout + pid->dout;
		bool sat_hi = (out_wo_new_i >= hi) && (pid->error[0] > 0.0f);
		bool sat_lo = (out_wo_new_i <= lo) && (pid->error[0] < 0.0f);
		if(!sat_hi && !sat_lo) {
			pid->iout += pid->i * pid->error[0];
			if(pid->iout >  i_limit) pid->iout =  i_limit;
			if(pid->iout < -i_limit) pid->iout = -i_limit;
		}
	} else {
		pid->iout = 0.0f;  /* 显式禁用积分，杜绝历史残留偏置 */
	}

	pid->out = pid->pout + pid->iout + pid->dout;

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];

	/* 只截断输出，不回写 iout。 */
	if(pid->out > hi) pid->out = hi;
	if(pid->out < lo) pid->out = lo;
}

void pidout_Servo_limit(pid_t *pid)
{
	PIDOUT_CLAMP(pid, -40.0f, 40.0f);
}





/*
void pidout_limit_Y(pid_t *pid)
{
	PIDOUT_CLAMP(pid, -50.0f, 50.0f);
}
*/
float motor_get_command_angle(void)
{
	return (float)motor_command_pulses / PULSES_PER_DEG;
}

void motor_set_angle(float target_deg)
{
	/* PID 输出是相对上电回零点的绝对目标倾角。 */
	if(target_deg > ANGLE_LIMIT_MAX) target_deg = ANGLE_LIMIT_MAX;
	if(target_deg < ANGLE_LIMIT_MIN) target_deg = ANGLE_LIMIT_MIN;

	/* 驱动器绝对位置模式：+θ 为零点上方，-θ 为零点下方。 */
	int32_t target_pulses = (int32_t)(target_deg * PULSES_PER_DEG);
	if(target_pulses == motor_command_pulses) {
		return;
	}

	if(target_pulses >= 0) {
		Emm_V5_Pos_Control(MOTOR_ADDR, 0, MOTOR_POSITION_SPEED_RPM, 0, (uint32_t)target_pulses, 1, 0);
	} else {
		Emm_V5_Pos_Control(MOTOR_ADDR, 1, MOTOR_POSITION_SPEED_RPM, 0, (uint32_t)(-target_pulses), 1, 0);
	}

	motor_command_pulses = target_pulses;
}

void pid_control__26Y(void)
{
	static float relative_y_last = 0.0f;
	static float target_y_last = 0.0f;
	static bool control_initialized = false;
	static bool target_hold_active = false;
	static uint8_t target_hold_settle_ticks = 0u;
	static float neg6_relative_y_filtered = 0.0f;
	static bool neg6_near_zone_active = false;

	if(!g_pid_segment_initialized) {
		pid_apply_segment_profile(5.6f, 1.735f, 0.0f, 0.0f, 0.99f, 0.01f, 0.0f);
		g_pid_segment = PID_SEGMENT_TO_POS_56;
		g_pid_segment_initialized = true;
	}

	Y = ball_error;  // 视觉小球绝对位置

	/* 分段状态机：
	 *   1) 先用第一组参数把球从 0cm 推到 +5.6cm；
	 *   2) 一旦进入 +5.6cm ±0.5cm，立刻切第二组参数并改目标到 -5.5cm；
	 *   3) 到达 -5.5cm ±0.5cm 后进入最终保持，只停位置环、速度环继续保 0 速度。 */
	if(g_pid_segment == PID_SEGMENT_TO_POS_56) {
		if(pid_absf(Y - 5.6f) <= TARGET_HOLD_POSITION_ENTER) {
			pid_apply_segment_profile(-6.0f, 0.750f, 0.0f, 0.0f, 1.0f, 0.1f, 0.0f);
			g_pid_segment = PID_SEGMENT_TO_NEG_55;
			control_initialized = false;
			target_hold_active = false;
			target_hold_settle_ticks = 0u;
			neg6_relative_y_filtered = 0.0f;
		}
	} else if(g_pid_segment == PID_SEGMENT_TO_NEG_55) {
		if(pid_absf(Y + 6.0f) <= TARGET_HOLD_POSITION_ENTER) {
			g_pid_segment = PID_SEGMENT_HOLD_NEG_55;
			control_initialized = false;
			target_hold_active = false;
			target_hold_settle_ticks = 0u;
		}
	}

	/* 在目标坐标系中控制：负值表示球在目标负侧，正值表示球在目标正侧。 */
	float target_y = ball_target_get();
	float relative_y = Y - target_y;
	bool target_changed = !control_initialized || target_y != target_y_last;

	/* 切换目标时丢弃旧状态，避免把上一个目标的补偿带到新目标。 */
	if(target_changed) {
		target_hold_active = false;
		target_hold_settle_ticks = 0u;
		relative_y_last = relative_y;
		target_y_last = target_y;
		control_initialized = true;
	}

	float position_delta = relative_y - relative_y_last;
	redYSpeed = position_delta;  // 保留旧诊断变量，单位为每控制周期 cm

	/* 内环速度反馈改为本地位置差分速度。
	 * MaixCAM 直接给出的速度存在大量 0/尖峰交替，容易让内环误判“已经超速”，
	 * 提前给出负倾角。位置量测本身连续得多，因此用 relative_y 差分更稳。
	 */
	float estimated_velocity = position_delta / PID_CONTROL_PERIOD_S;
	float abs_relative_y = (relative_y >= 0.0f) ? relative_y : -relative_y;
	float abs_estimated_velocity = (estimated_velocity >= 0.0f) ? estimated_velocity : -estimated_velocity;
	bool target_is_neg6 = (target_y == -6.0f);
	float target_hold_enter = target_is_neg6 ? 0.18f : TARGET_HOLD_POSITION_ENTER;
	float target_hold_exit = target_is_neg6 ? 0.28f : TARGET_HOLD_POSITION_EXIT;

	/* -6.0cm 终点补丁：不再切新的分段 PID，而是在近端直接保护观测与压小目标速度。
	 * 1) |error|<=1.2cm 时，若单帧位置跳变超过 1.0cm，则判为视觉毛刺，本周期沿用上一帧 relative_y；
	 * 2) 对近端 relative_y 再做一阶低通，但滤波更快一些，减少末端相位滞后。 */
	if(target_is_neg6) {
		float relative_y_for_control = relative_y;
		if(abs_relative_y <= 1.2f && pid_absf(position_delta) > 1.0f) {
			relative_y_for_control = relative_y_last;
		}
		neg6_relative_y_filtered += 0.45f * (relative_y_for_control - neg6_relative_y_filtered);
		relative_y = neg6_relative_y_filtered;
		position_delta = relative_y - relative_y_last;
		estimated_velocity = position_delta / PID_CONTROL_PERIOD_S;
		abs_relative_y = pid_absf(relative_y);
		abs_estimated_velocity = pid_absf(estimated_velocity);
	} else {
		neg6_relative_y_filtered = relative_y;
	}

	/* -6.0 近端状态机：首次进入 |error|<=1.0cm 时清一次速度环积分，
	 * 避免把回程累积的负向积分带进终点区，导致过点后仍持续猛推；
	 * 直到明显退出到 1.4cm 以外才复位，留出回差防止边界抖动。 */
	if(target_is_neg6) {
		if(!neg6_near_zone_active && abs_relative_y <= 1.0f) {
			neg6_near_zone_active = true;
			pidY_Speed.iout = 0.0f;
		} else if(neg6_near_zone_active && abs_relative_y > 1.4f) {
			neg6_near_zone_active = false;
		}
	} else {
		neg6_near_zone_active = false;
	}

	/* 最终保持只在第二段完成后生效。
	 * 第一段到 +5.6cm 时不做停留，而是立刻切第二组参数继续向 -5.5cm 运动。
	 *
	 * 进入最终保持不能只看位置，还必须确认球已经基本停住；否则会在“带着余速穿过
	 * 目标窗”的瞬间提前锁定，随后又滑出保持区，形成反复进出与停错位置。 */
	if(g_pid_segment == PID_SEGMENT_HOLD_NEG_55) {
		if(target_hold_active) {
			if(abs_relative_y >= target_hold_exit) {
				target_hold_active = false;
				target_hold_settle_ticks = 0u;
			}
		} else if(abs_relative_y <= target_hold_enter &&
		          abs_estimated_velocity <= TARGET_HOLD_SPEED_LIMIT) {
			if(target_hold_settle_ticks < TARGET_HOLD_SETTLE_TICKS) {
				target_hold_settle_ticks++;
			}
			if(target_hold_settle_ticks >= TARGET_HOLD_SETTLE_TICKS) {
				target_hold_active = true;
			}
		} else {
			target_hold_settle_ticks = 0u;
		}
	} else {
		target_hold_active = false;
		target_hold_settle_ticks = 0u;
	}

	if(target_hold_active) {
		/* 进入误差保持区后，只停止位置环继续推球：
		 *   1) 外环目标速度稍后压到 0；
		 *   2) 速度环继续按“目标速度 = 0”闭环抑制残余滚动。 */
	}

	/* PID 固定跟踪相对坐标系的零点，ball_target_y 可随时更改。 */
	pidY.target = 0.0f;
	pidY.now = relative_y;
	/* 外环用条件积分限幅。POS_LOOP_I_LIMIT=0 表示外环不积分（稳态误差交给内环积分消除），
	 * 且每周期强制 iout=0，即使曾被历史代码写坏也会立刻清掉。
	 * 绝不能再用 PIDOUT_CLAMP：外环 pout 在 |relative_y|>12.5cm 时超过 5.0 限幅，
	 * 反算抗饱和会把溢出量永久写进 iout，Ki=0 时再也退不回来，
	 * 表现为 ch3 恒定偏低一个常数（实测 −2.245 cm/s），球停在错误位置。 */
	pid_cal_clamped(&pidY, -BALL_TARGET_SPEED_LIMIT, BALL_TARGET_SPEED_LIMIT,
	                POS_LOOP_I_LIMIT);
	if(target_is_neg6 && !target_hold_active) {
		/* 从 +5.6 回到 -6.0 的中段仍保留巡航托底，避免太早掉速；
		 * 但终点前的“硬托底”只保留到 0.45cm，避免在 0.5~0.8cm 这段先收得太软又突然猛拉。 */
		if(relative_y > 2.8f) {
			if(pidY.out > -4.2f) pidY.out = -4.2f;
		} else if(relative_y > 0.45f) {
			if(pidY.out > -2.4f) pidY.out = -2.4f;
		}
	}
	if(target_is_neg6) {
		/* -6.0 前最后一小段根据近端状态进一步收口。
		 * 进入近端后不再允许外环继续给太大的负目标速度，避免把球直接打穿终点。 */
		if(neg6_near_zone_active) {
			float neg6_speed_limit = 0.22f;
			if(relative_y > 0.22f) neg6_speed_limit = 0.45f;
			if(estimated_velocity < -1.6f) {
				neg6_speed_limit = 0.08f;
			} else if(estimated_velocity < -0.9f) {
				neg6_speed_limit = 0.14f;
			}
			if(pidY.out > neg6_speed_limit) pidY.out = neg6_speed_limit;
			else if(pidY.out < -neg6_speed_limit) pidY.out = -neg6_speed_limit;
		} else if(abs_relative_y <= 0.22f) {
			float neg6_speed_limit = 0.18f;
			if(pidY.out > neg6_speed_limit) pidY.out = neg6_speed_limit;
			else if(pidY.out < -neg6_speed_limit) pidY.out = -neg6_speed_limit;
		}
	}
	if(target_hold_active) {
		/* 进入保持后，对 -6.0 不再用固定 ±0.22 的硬切换，
		 * 改成一个很小的比例型目标速度，让内环持续压残余滚动但少一点反复抽动。 */
		if(target_is_neg6) {
			if(pid_absf(relative_y) <= 0.06f) {
				pidY.out = 0.0f;
			} else {
				pidY.out = -1.1f * relative_y;
				if(pidY.out > 0.10f) pidY.out = 0.10f;
				else if(pidY.out < -0.10f) pidY.out = -0.10f;
			}
		} else {
			pidY.out = 0.0f;
		}
	}

	/* 先限制本地估计速度，再低通滤波，VOFA ch5 继续保留这个诊断量。
	 * 单调位置环时它不参与控制，只作为观察球在平台上实际运动情况的参考。
	 */
	float control_velocity = estimated_velocity;
	if(control_velocity > BALL_VELOCITY_CONTROL_LIMIT) {
		control_velocity = BALL_VELOCITY_CONTROL_LIMIT;
	} else if(control_velocity < -BALL_VELOCITY_CONTROL_LIMIT) {
		control_velocity = -BALL_VELOCITY_CONTROL_LIMIT;
	}
	pidY_filtered_velocity += BALL_VELOCITY_FILTER_ALPHA *
	                         (control_velocity - pidY_filtered_velocity);

#if POSITION_LOOP_DIRECT_DRIVE
	/* 单环调试模式：位置环直接输出平台倾角。
	 * 位置环此时直接调“位置 -> 角度”，便于先把 P/I/D 基本手感调顺。
	 * 仍复用电机最终角度限幅与角度斜坡限制，避免机械冲击。 */
	pidY_velocity_damping = 0.0f;
	pidY_Speed.target = 0.0f;
	pidY_Speed.now = pidY_filtered_velocity;
	pidY_Speed.out = 0.0f;
	pidY_Speed.pout = 0.0f;
	pidY_Speed.iout = 0.0f;
	pidY_Speed.dout = 0.0f;

	float cmd_angle = pidY.out;
	#else
		/* 外环给出小球目标速度，内环以本地估计速度为反馈输出平台倾角。 */
		pidY_Speed.target = pidY.out;
		pidY_Speed.now = pidY_filtered_velocity;
		pid_cal_clamped(&pidY_Speed, -SPEED_LOOP_ANGLE_LIMIT, SPEED_LOOP_ANGLE_LIMIT,
		                SPEED_LOOP_I_LIMIT);
		pidY_velocity_damping = pidY_Speed.out;  /* 保留旧变量名，供 VOFA 观察限幅后的速度环输出 */
		float cmd_angle = pidY_velocity_damping;
	#endif

	if(target_is_neg6) {
		/* 终点附近最明显的问题已经不是外环 target speed，而是速度环会把倾角打得过猛。
		 * 限角要比外环收口更早生效，否则第一脚大角度已经把球打穿终点了。 */
		float neg6_angle_limit = SPEED_LOOP_ANGLE_LIMIT;
		if(abs_relative_y <= 1.2f) neg6_angle_limit = 4.5f;
		if(abs_relative_y <= 0.75f) neg6_angle_limit = 3.2f;
		if(neg6_near_zone_active) neg6_angle_limit = 1.8f;
		if(target_hold_active) neg6_angle_limit = 1.2f;
		if(cmd_angle > neg6_angle_limit) cmd_angle = neg6_angle_limit;
		else if(cmd_angle < -neg6_angle_limit) cmd_angle = -neg6_angle_limit;
	}

	// 负方向（下降）机构补偿 ×1.2。
	if(cmd_angle < 0.0f) {
		cmd_angle *= 1.0f;
	}

	/* 倾角变化率限制：每 20ms tick 最大变化 ANGLE_SLEW_PER_TICK，
	 * 避免 PID 在相邻周期间从 +限幅 直接跳到 -限幅 造成机械冲击振荡。 */
	static float cmd_angle_last = 0.0f;
	{
		float delta = cmd_angle - cmd_angle_last;
		if(delta >  ANGLE_SLEW_PER_TICK) delta =  ANGLE_SLEW_PER_TICK;
		else if(delta < -ANGLE_SLEW_PER_TICK) delta = -ANGLE_SLEW_PER_TICK;
		cmd_angle = cmd_angle_last + delta;
		cmd_angle_last = cmd_angle;
	}
	motor_set_angle(cmd_angle);

	relative_y_last = relative_y;
	Y_last = Y;
}

void pidout_limit_Y(pid_t *pid)
{
	PIDOUT_CLAMP(pid, -50.0f, 50.0f);
}