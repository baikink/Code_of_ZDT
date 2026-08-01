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
  POSITION_PID = 0,  // ¦Ë???
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
extern float Y;
extern float Y_last;

extern pid_t pidY;
extern pid_t pidY_Speed;

void pid_control__26Y(void);

void pidout_limit_Y(pid_t *pid);







#endif
