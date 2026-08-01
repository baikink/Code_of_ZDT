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

/* 输出限幅宏（�?抗积分饱和）
 * 超出 [lo, hi] 时同步把超出量从 iout 里减回来，防止积分越界累�?导致抖动
 * 用法：PIDOUT_CLAMP(&pidXxx, -50.0f, 50.0f);
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
#define ANGLE_LIMIT_MAX    6.0f    /* 正向最大角度 (°) */
#define ANGLE_LIMIT_MIN  -10.0f    /* 反向最大角度 (°) */

/* 视觉速度阻尼：先低通滤波，再将速度换算为反向制动倾角。 */
#define BALL_VELOCITY_FILTER_ALPHA  0.25f
#define VELOCITY_DAMPING_K           0.12f   /* °/(cm/s) */
#define VELOCITY_DAMPING_LIMIT       3.0f    /* 最大制动倾角 (°) */

/* 小球偏离中心但持续不动时，用最小倾角克服静摩擦。 */
#define STUCK_POSITION_THRESHOLD     0.5f    /* 仅在 |Y| > 0.5 cm 时检测卡滞 */
#define STUCK_POSITION_DELTA         0.03f   /* 单控制周期内的位置变化阈值 (cm) */
#define STUCK_TICKS_REQUIRED         10u     /* 连续 10 × 20ms = 200ms 不动 */
#define BREAKAWAY_POS_ANGLE          4.5f    /* Y < 0 时的正向起动倾角 (°) */
#define BREAKAWAY_NEG_ANGLE         -3.2f    /* Y > 0 时的负向起动倾角，后续再 ×1.2 */
#define BREAKAWAY_RELEASE_DISTANCE   0.25f   /* 向中心累计移动该距离后退出起动补偿 (cm) */

extern float Y;
extern float Y_last;

extern pid_t pidY;
extern pid_t pidY_Speed;
extern float pidY_velocity_damping;

void pid_control__26Y(void);
void motor_set_angle(float target_deg);
float motor_get_command_angle(void);

void pidout_limit_Y(pid_t *pid);







#endif
