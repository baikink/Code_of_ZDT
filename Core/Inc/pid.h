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

extern float Y;
extern float Y_last;

extern pid_t pidY;
extern pid_t pidY_Speed;

void pid_control__26Y(void);
void motor_set_angle(float target_deg);
float motor_get_command_angle(void);

void pidout_limit_Y(pid_t *pid);







#endif
