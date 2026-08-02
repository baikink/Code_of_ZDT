#ifndef __PID_h_
#define __PID_h_
#include <stdint.h>
#include <stdbool.h>


extern int redX_last;
extern int redY_last;
extern int redXSpeed;
extern int redYSpeed;

enum
{
  POSITION_PID = 0,  // ��???
  DELTA_PID,         // ?????
};

typedef struct
{
	float target;	
	float target_last;
	float now;
	float error[3];		
	float error_taget;	
	float p,i,d,f;
	float pout, dout, iout,fout;
	float out;   
	
	uint32_t pid_mode;

}pid_t;

/* 输出限幅宏（反算式抗积分饱和）
 * 超出 [lo, hi] 时把超出量从 iout 里减回来。
 * 用法：PIDOUT_CLAMP(&pidXxx, -50.0f, 50.0f);
 *
 * ⚠️ 仅适用于「饱和确实由积分项造成」的场合。若比例项单独就能大幅越限
 *    （小球平衡内环：error 可达 ±13cm/s × P1.1 = ±14° vs 4° 限幅），
 *    反算会把**比例项的溢出量**写进 iout：
 *      - Ki = 0 时该偏置永久不化，环路从此带一个常值反向偏置；
 *      - Ki > 0 时也要数秒才能爬回，期间持续反顶外环。
 *    串级环请改用 pid.c 的 pid_cal_clamped()（条件积分，绝不污染 iout）。
 */
#define PIDOUT_CLAMP(pid, lo, hi)                               \
    do {                                                        \
        if((pid)->out > (float)(hi)) {                          \
            (pid)->iout -= ((pid)->out - (float)(hi));          \
            (pid)->out   = (float)(hi);                         \
        }                                                       \
        if((pid)->out < (float)(lo)) {                          \
            (pid)->iout -= ((pid)->out - (float)(lo));          \
            (pid)->out   = (float)(lo);                         \
        }                                                       \
    } while(0)

/*extern pid_t motorA;
extern pid_t motorB;
extern pid_t motorC;
extern pid_t motorD;

extern pid_t weizhiA;
extern pid_t weizhiB;
extern pid_t weizhiC;
extern pid_t weizhiD;

extern pid_t ServoA;
extern pid_t ServoB;*/

/*extern pid_t ServoA_Speed;
extern pid_t ServoB_Speed;

extern pid_t ServoA_Pid;
extern pid_t ServoB_Pid;*/

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d,float f);
void pid_cal(pid_t *pid);
/*void motor_target_set(int spe1, int spe2);*/
void pidout_limit(pid_t *pid);

/*void pid_control(void);
void pid_control_speed(void);
void pid_control_speed_CD(void);
void pid_control_Position_CD(int C,int D);
void pid_control_Speed_Position_CD(int C,int D,int SC,int SD);
void pid_control_Speed_Position_AB(int A,int B,int SA,int SB);*/

/*void pid_control_X(void);
void pid_control_Y(void);*/


void pidout_Servo_limit(pid_t *pid);

////////////////////26//////////////
/* ── 电机角度限幅（基于上电零点），只需改这两个数字 ── */
#define ANGLE_LIMIT_MAX   12.0f    /* 正向最终机械保护角度 (°) */
#define ANGLE_LIMIT_MIN  -12.0f    /* 反向最终机械保护角度 (°) */

/* 串级控制的正常工作范围；最终机械限幅仅作为最后一级保护。 */
#define POSITION_LOOP_DIRECT_DRIVE    0       /* 1: 位置环直接输出电机角度，便于单独调位置环；0: 恢复串级 PID */
#define BALL_VELOCITY_CONTROL_LIMIT  8.0f  /* 进入速度环前的速度反馈限幅 (cm/s) */

/* 外位置环输出的目标速度限幅 (cm/s)。
 * 必须满足 限幅 ≈ 外环P × 位置半量程（0.40 × 12.5 ≈ 5.0），否则误差一超过
 * 限幅/P 的距离，外环就恒定饱和，位置信息丢失，串级退化成纯调速器。
 * 曾用 1.5f：外环在 |error| > 3.75cm 即饱和，球从远处滚来时目标速度恒为
 * 1.5cm/s，而实际下滑速度可达 6cm/s，内环立刻输出满量程负角提前刹车，
 * 球在离目标还有 10cm 处就被摁住甚至被推回，无法抵达目标。不要再调回该值。
 */
#define BALL_TARGET_SPEED_LIMIT       10.0f
#define SPEED_LOOP_ANGLE_LIMIT        10.0f  /* 内速度环输出的正常倾角限幅 (°) */

/* 内速度环积分限幅 (°)：积分项唯一职责是补偿"命令 0° 并非真正水平"的机械零点
 * 偏差与滚动摩擦，因此上限取略小于正常倾角限幅，避免积分独自吃满整个工作范围。
 */
#define SPEED_LOOP_I_LIMIT            2.5f

/* 外位置环积分限幅 (cm/s)：外环目前 Ki=0，故取 0 —— 配合 pid_cal_clamped()
 * 可保证 iout 恒为 0，任何饱和都不会在 Ki=0 的环上留下永久偏置。
 * 若将来给外环加积分，把这里改成非零上限即可。
 */
#define POS_LOOP_I_LIMIT              0.0f

/* 倾角变化率限制：限制每个控制周期(20ms)命令角度的最大变化量，
 * 仅用于避免相邻周期从 +限幅 直接跳到 -限幅 的机械冲击，不应成为主要滞后来源。
 * 1.0°/20ms = 50°/s：全量程 ±4° 换向约需 160ms，仍远快于小球动力学。
 * 注意：0.15°/tick(7.5°/s) 会让 4° 命令需要 540ms 才到位，
 * 相位滞后远超小球响应时间，会直接导致越过目标后无法收敛，不要再调回该值。
 */
#define ANGLE_SLEW_PER_TICK          1.0f

/* 控制周期与本地速度估计。
 * 主循环实际每 20ms 调一次控制，因此直接用位置差分 / 0.02s 估计球速。
 * 位置量测比 MaixCAM 直接给出的速度更连续稳定，适合作为内环反馈。
 */
#define PID_CONTROL_PERIOD_S        0.02f

/* 本地估计速度低通：滤波后的速度作为串级速度环的当前值。 */
#define BALL_VELOCITY_FILTER_ALPHA  0.25f

/* 接近目标后的稳态区。
 *
 * 需求改为：|位置误差| <= 0.5cm 时就停止 PID 调节。
 * 为避免视觉噪声和轻微回弹导致反复进出保持区，退出门限留 0.1cm 回差，
 * 即误差重新增大到 0.6cm 以上才恢复闭环。
 */
#define TARGET_HOLD_POSITION_ENTER  0.50f
#define TARGET_HOLD_POSITION_EXIT   0.60f
#define TARGET_HOLD_SPEED_LIMIT     0.25f

/* 小球偏离目标且持续不动时，用起动倾角克服静摩擦。
 *
 * 不再使用固定单一的 breakaway 角。
 * 现改为“分级加力”：先给一个较小起动角，若球仍不动，就每隔一段时间再加一点，
 * 直到上限。这样既能在轻微卡滞时保持动作柔和，也能在重静摩擦时继续加力，避免
 * 像之前那样一直卡死在固定的 -3.6° 上。
 */
#define STUCK_POSITION_THRESHOLD      0.6f    /* 仅在距目标 |error| > 0.6 cm 时检测卡滞并启用起动倾角，避免 0.6~1.0cm 区间卡死 */
#define STUCK_POSITION_DELTA          0.03f   /* 单控制周期内、相对目标的位置变化阈值 (cm) */
#define STUCK_TICKS_REQUIRED          10u     /* 连续 10 × 20ms = 200ms 不动后进入 breakaway */
#define BREAKAWAY_POS_ANGLE_BASE      2.0f    /* 正向起动角初值 (°) */
#define BREAKAWAY_NEG_ANGLE_BASE     -3.0f    /* 负向起动角初值 (°)，后续再 ×1.2 */
#define BREAKAWAY_POS_ANGLE_MAX       4.0f    /* 正向起动角上限 (°) */
#define BREAKAWAY_NEG_ANGLE_MAX      -5.0f    /* 负向起动角上限 (°)，后续再 ×1.2 */
#define BREAKAWAY_ANGLE_STEP          0.5f    /* 每次加力的步长 (°) */
#define BREAKAWAY_RAMP_TICKS          8u      /* 每 8 × 20ms = 160ms 仍不动则再加一级 */
#define BREAKAWAY_RELEASE_DISTANCE    0.25f   /* 向目标累计移动该距离后退出起动补偿 (cm) */

extern float Y;
extern float Y_last;

/* 小球目标位置，单位 cm，坐标系与 Y 保持一致；可在运行中随时更新。 */
extern volatile float ball_target_y;
void ball_target_set(float target_y);
float ball_target_get(void);

extern pid_t pidY;        /* 单环直驱时：位置 -> 平台倾角；串级时：位置 -> 小球目标速度 */
extern pid_t pidY_Speed;  /* 串级模式下：小球速度 -> 平台倾角 */
extern float pidY_velocity_damping;  /* 保留旧变量名，当前为速度环倾角输出 */
extern float pidY_filtered_velocity; /* 低通滤波后的视觉速度 (cm/s)，供 VOFA ch5 */

void pid_control__26Y(void);
void motor_set_angle(float target_deg);
float motor_get_command_angle(void);

void pidout_limit_Y(pid_t *pid);







#endif
