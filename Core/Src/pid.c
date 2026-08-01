#include "pid.h"
#include "Emm_V5.h"              /* ball_error、Emm_V5_Pos_Control */

extern float ball_velocity;  /* 视觉小球速度，定义在 main.c */

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

/*pid_t motorA;
pid_t motorB;
pid_t motorC;
pid_t motorD;

pid_t weizhiA;
pid_t weizhiB;
pid_t weizhiC;
pid_t weizhiD;

pid_t ServoA;
pid_t ServoB;

pid_t ServoA_Speed;
pid_t ServoB_Speed;

pid_t ServoA_Pid;
pid_t ServoB_Pid;

int redX_last;
int redY_last;*/

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

/*void motor_target_set(int spe1, int spe2)
{
	if(spe1 >= 0)
	{
		motorA_dir = 1;
		motorA.target = spe1;
	}
	else
	{
		motorA_dir = 0;
		motorA.target = -spe1;
	}
	
	if(spe2 >= 0)
	{
		motorB_dir = 1;
		motorB.target = spe2;
	}
	else
	{
		motorB_dir = 0;
		motorB.target = -spe2;
	}
}

void motor_target_set_CD(int spe1, int spe2)
{
	if(spe1 >= 0)
	{
		motorC_dir = 1;
		motorC.target = spe1;
	}
	else
	{
		motorC_dir = 0;
		motorC.target = -spe1;
	}
	
	if(spe2 >= 0)
	{
		motorD_dir = 1;
		motorD.target = spe2;
	}
	else
	{
		motorD_dir = 0;
		motorD.target = -spe2;
	}
}

//void Servo_target_set(int spe1, int spe2)
//{
//	ServoA.target = spe1;
//	ServoB.target = spe2;
//	
//	
//}

void pid_control(void)
{
	// 1.�趨Ŀ���ٶ�
//	motorA.target = 50;
	motor_target_set(30,30);
	// 2.��ȡ��ǰ�Ƕ�
	if(motorA_dir){motorA.now = speedA;}else{motorA.now = -speedA;}
	if(motorB_dir){motorB.now = speedB;}else{motorB.now = -speedB;}
	Encoder_count1 = 0;
	Encoder_count2 = 0;
	// 3.PID�������������
	pid_cal(&motorA);
	pid_cal(&motorB);
	// �������޷�
	pidout_limit(&motorA);
	pidout_limit(&motorB);
	// 4.PID�����ֵ ��������
	motorA_duty(motorA.out);
	motorB_duty(motorB.out);
}

void pid_control_speed(void)
{
	// 1.�趨Ŀ���ٶ�
//	motorA.target = 50;
	motor_target_set(30,30);
	// 2.��ȡ��ǰ�Ƕ�
	if(motorA_dir){motorA.now = speedA;}else{motorA.now = -speedA;}
	if(motorB_dir){motorB.now = speedB;}else{motorB.now = -speedB;}
	Encoder_count1 = 0;
	Encoder_count2 = 0;
	// 3.PID�������������
	pid_cal(&motorA);
	pid_cal(&motorB);
	// �������޷�
	pidout_limit(&motorA);
	pidout_limit(&motorB);
	// 4.PID�����ֵ ��������
	motorA_duty(motorA.out);
	motorB_duty(motorB.out);
}

void pid_control_speed_CD(void)
{
	// 1.�趨Ŀ���ٶ�
//	motorA.target = 50;
	motor_target_set_CD(200,200);
	// 2.��ȡ��ǰ�Ƕ�
	if(motorC_dir){motorC.now = speedC;}else{motorC.now = -speedC;}
	if(motorD_dir){motorD.now = speedD;}else{motorD.now = -speedD;}
//	if(motorC_dir){motorC.now = speedC;}else{motorC.now = -speedC;}
//	if(motorD_dir){PositionD = (((Encoder_Timer_Overflow_D-1)*65535)+(TIM_GetCounter(TIM5)));}
//	else{PositionD = -(-(Encoder_Timer_Overflow_D*65535)+(65535-TIM_GetCounter(TIM5)));}
	Encoder_count3	= 0;
	Encoder_count4 = 0;
	// 3.PID�������������
	pid_cal(&motorC);
	pid_cal(&motorD);
	// �������޷�
	pidout_limit(&motorC);
	pidout_limit(&motorD);
	// 4.PID�����ֵ ��������
	motorC_duty(motorC.out);
	motorD_duty(motorD.out);
}

void pid_control_Position_CD(int C,int D)
{
	// 1.�趨Ŀ��λ��
	weizhiC.target = C;
	weizhiD.target = D;
	//2��ȡ��ǰλ��
	weizhiC.now = PositionC;
	weizhiD.now = PositionD;
	//pid����
	pid_cal(&weizhiC);
	pid_cal(&weizhiD);
	//���뵽���
	motorC_duty(weizhiC.out);
	motorD_duty(weizhiD.out);
	
}

void pid_control_Speed_Position_CD(int C,int D,int SC,int SD)
{
	// 1.�趨Ŀ��λ��
	weizhiC.target = C;
	weizhiD.target = D;
	//2��ȡ��ǰλ��
	weizhiC.now = PositionC;
	weizhiD.now = PositionD;
	//pid����
	pid_cal(&weizhiC);
	pid_cal(&weizhiD);
	//�����ٶȻ�
	// 1.�趨Ŀ���ٶ�	
	if(weizhiC.out >= SC)weizhiC.out = SC;
	if(weizhiD.out >= SD)weizhiD.out = SD;	
	motor_target_set_CD(weizhiC.out,weizhiD.out);
	
	
	// 2.��ȡ��ǰ�ٶ�
	if(motorC_dir){motorC.now = speedC;}else{motorC.now = -speedC;}
	if(motorD_dir){motorD.now = speedD;}else{motorD.now = -speedD;}
	// 3.PID�������������
	pid_cal(&motorC);
	pid_cal(&motorD);
	// �������޷�
	pidout_limit(&motorC);
	pidout_limit(&motorD);
	// 4.PID�����ֵ ��������
	if((weizhiC.target - weizhiC.now)<=10 && (weizhiC.target - weizhiC.now) >=0)
		motorC_duty(0);
	else if((weizhiC.target - weizhiC.now)>=-10 && (weizhiC.target - weizhiC.now) <= 0)
		motorC_duty(0);
	else
		motorC_duty(motorC.out);
	
	if((weizhiD.target - weizhiD.now)<=10 && (weizhiD.target - weizhiD.now) >=0)
		motorD_duty(0);
	else if((weizhiD.target - weizhiD.now)>=-10 && (weizhiD.target - weizhiD.now) <= 0)
		motorD_duty(0);
	else
		motorD_duty(motorD.out);
//	else
//		motorC_duty(motorC.out);
//	if((weizhiD.target - weizhiC.now)>=-10 || (weizhiC.target - weizhiC.now) <= 0)
//		motorD_duty(0);
//	else
//		motorD_duty(motorD.out);	
	
}

void pid_control_Speed_Position_AB(int A,int B,int SA,int SB)
{
	// 1.�趨Ŀ��λ��
	weizhiA.target = A;
	weizhiB.target = B;
	//2��ȡ��ǰλ��
	weizhiA.now = PositionA;
	weizhiB.now = PositionB;
	//pid����
	pid_cal(&weizhiA);
	pid_cal(&weizhiB);
	//�����ٶȻ�
	// 1.�趨Ŀ���ٶ�	
	if(weizhiA.out >= SA)weizhiA.out = SA;
	if(weizhiB.out >= SB)weizhiB.out = SB;	
	motor_target_set(weizhiA.out,weizhiB.out);
	
	
	// 2.��ȡ��ǰ�ٶ�
	if(motorA_dir){motorA.now = speedA;}else{motorA.now = -speedC;}
	if(motorB_dir){motorB.now = speedB;}else{motorB.now = -speedB;}
	// 3.PID�������������
	pid_cal(&motorA);
	pid_cal(&motorB);
	// �������޷�
	pidout_limit(&motorA);
	pidout_limit(&motorB);
	// 4.PID�����ֵ ��������
	if((weizhiA.target - weizhiA.now)<=10 && (weizhiA.target - weizhiA.now) >=0)
		motorA_duty(0);
	 else if((weizhiA.target - weizhiA.now)>=-10 && (weizhiA.target - weizhiA.now) <= 0)
		motorA_duty(0);
	else
		motorA_duty(motorA.out);
	
	if((weizhiB.target - weizhiB.now)<=10 && (weizhiB.target - weizhiB.now) >=0)
		motorB_duty(0);
	else if((weizhiB.target - weizhiB.now)>=-10 && (weizhiB.target - weizhiB.now) <= 0)
		motorB_duty(0);
	else
		motorB_duty(motorB.out);
//	else
//		motorC_duty(motorC.out);
//	1if((weizhiD.target - weizhiC.now)>=-10 || (weizhiC.target - weizhiC.now) <= 0)
//		motorD_duty(0);
//	else
//		motorD_duty(motorD.out);	
	
}


//void pid_control_X(void)
//{
//	redXSpeed = red_value[0] - redX_last;
//	
//	// 1.�趨Ŀ��λ��
//	ServoA.target =  70;//rect_value[0]
//	// 2.��ȡ��ǰλ��
//	ServoA.now = red_value[0];
////	if(ServoA.target - ServoA.now <= 1 && ServoA.target - ServoA.now >=0)
////	{
////		ServoA.target = ServoA.now;
////	}
////	if(ServoA.now - ServoA.target >= -1 && ServoA.now - ServoA.target <=0)
////	{
////		ServoA.target = ServoA.now;
////	}
//	
//	// 3.PID�������������
//	pid_cal(&ServoA);
////	if(ServoA.out < 0)ServoA.out = -ServoA.out;
//	
//	// �������޷�
//	pidout_Servo_limit(&ServoA);
//	
//	// 4.PID�����ֵ ��������
//	
//	ServoA_Speed.target = ServoA.out;
//	
//	ServoA_Speed.now = redXSpeed;

//	pid_cal(&ServoA_Speed);
//	pidout_Servo_limit(&ServoA_Speed);
//	
//	if(ServoA_Speed.out < 0)ServoA_Speed.out = -ServoA_Speed.out;	
//	
//	//�������pid
////	Servo_SetAngleA_2(ServoA_Speed.out);
//	ServoA_Pid.target = ServoA_Speed.out;
//	
//	ServoA_Pid.now =ServoA_Pid.now;
//	
//	if(ServoA_Pid.now>=ServoA_Pid.target && ServoA_Pid.target - ServoA_Pid.now > 0)ServoA_Pid.now=ServoA_Pid.target;
//	if(ServoA_Pid.now<=0)ServoA_Pid.now=0;
//	
//	pid_cal(&ServoA_Pid);
//	
//	if(ServoA_Pid.out>0.1f)ServoA_Pid.out=0.1f;
//	else if(ServoA_Pid.out<-0.1f)ServoA_Pid.out=-0.1f;
//	
//	Servo_SetAngleA_2(ServoA_Pid.now);
//	
//	redX_last = red_value[0];
//	
//}

//void pid_control_X(void)
//{
//	redXSpeed = red_value[1] - redX_last;
//	
//	// 1.�趨Ŀ��λ��
//	ServoA.target =  86;//rect_value[0]
//	// 2.��ȡ��ǰλ��
//	ServoA.now = red_value[1];
////	if(ServoA.target - ServoA.now <= 1 && ServoA.target - ServoA.now >=0)
////	{
////		ServoA.target = ServoA.now;
////	}
////	if(ServoA.now - ServoA.target >= -1 && ServoA.now - ServoA.target <=0)
////	{
////		ServoA.target = ServoA.now;
////	}
//	
//	// 3.PID�������������
//	pid_cal(&ServoA);
////	if(ServoA.out < 0)ServoA.out = -ServoA.out;
//	
//	// �������޷�
//	pidout_Servo_limit(&ServoA);
//	
//	// 4.PID�����ֵ ��������
////	if(ServoA.target - ServoA.now <= 10 && ServoA.target - ServoA.now >= 0)ServoA.out =0;
////	if(ServoA.target - ServoA.now >= -10 && ServoA.target - ServoA.now <= 0)ServoA.out =0;
//	
//	
////	Servo_SetAngleA_2(ServoA.out);
//	
//	ServoA_Speed.target = 0;
//	
//	ServoA_Speed.now = redXSpeed;

//	pid_cal(&ServoA_Speed);
//	pidout_Servo_limit(&ServoA_Speed);
////	
////	if(ServoA_Speed.out < 0)ServoA_Speed.out = -ServoA_Speed.out;	
////	
////	//�������pid
//	Servo_SetAngleA_2(ServoA_Speed.out);
////	ServoA_Pid.target = ServoA_Speed.out;
////	
////	ServoA_Pid.now =ServoA_Pid.now;
////	
////	if(ServoA_Pid.now>=ServoA_Pid.target && ServoA_Pid.target - ServoA_Pid.now > 0)ServoA_Pid.now=ServoA_Pid.target;
////	if(ServoA_Pid.now<=0)ServoA_Pid.now=0;
////	
////	pid_cal(&ServoA_Pid);
////	
////	if(ServoA_Pid.out>0.1f)ServoA_Pid.out=0.1f;
////	else if(ServoA_Pid.out<-0.1f)ServoA_Pid.out=-0.1f;
////	
////	Servo_SetAngleA_2(ServoA_Pid.now);
////	
//	redX_last = red_value[1];
//	
//}

void pid_control_X(void)
{
	redXSpeed = red_value[1] - redX_last;
	
	// 1.�趨Ŀ��λ��
	ServoA.target =  45;//rect_value[0]
	// 2.��ȡ��ǰλ��
	ServoA.now = red_value[1];
//	if(ServoA.target - ServoA.now <= 1 && ServoA.target - ServoA.now >=0)
//	{
//		ServoA.target = ServoA.now;
//	}
//	if(ServoA.now - ServoA.target >= -1 && ServoA.now - ServoA.target <=0)
//	{
//		ServoA.target = ServoA.now;
//	}
	
	// 3.PID�������������
	pid_cal(&ServoA);
//	if(ServoA.out < 0)ServoA.out = -ServoA.out;
	
	// �������޷�
	pidout_Servo_limit(&ServoA);
	
	// 4.PID�����ֵ ��������
//	if(ServoA.target - ServoA.now <= 3 && ServoA.target - ServoA.now >= 0)ServoA.out =0;
//	if(ServoA.target - ServoA.now >= -3 && ServoA.target - ServoA.now <= 0)ServoA.out =0;
	
	
//	Servo_SetAngleA_2(ServoA.out);
	
	ServoA_Speed.target = 0;//ServoA.out
	
	ServoA_Speed.now = redXSpeed;

	pid_cal(&ServoA_Speed);
	pidout_Servo_limit(&ServoA_Speed);
//	
//	if(ServoA.target - ServoA.now <= 3 && ServoA.target - ServoA.now >= 0)ServoA_Speed.out =0;
//	if(ServoA.target - ServoA.now >= -3 && ServoA.target - ServoA.now <= 0)ServoA_Speed.out =0;
//	
//	//�������pid
	Servo_SetAngleA_2(ServoA_Speed.out);
//	ServoA_Pid.target = ServoA_Speed.out;
//	
//	ServoA_Pid.now =ServoA_Pid.now;
//	
//	if(ServoA_Pid.now>=ServoA_Pid.target && ServoA_Pid.target - ServoA_Pid.now > 0)ServoA_Pid.now=ServoA_Pid.target;
//	if(ServoA_Pid.now<=0)ServoA_Pid.now=0;
//	
//	pid_cal(&ServoA_Pid);
//	
//	if(ServoA_Pid.out>0.1f)ServoA_Pid.out=0.1f;
//	else if(ServoA_Pid.out<-0.1f)ServoA_Pid.out=-0.1f;
//	
//	Servo_SetAngleA_2(ServoA_Pid.now);
//	
	redX_last = red_value[1];
	
}

//void pid_control_Y(void)
//{
//	redYSpeed = red_value[1] - redY_last;
//	
//	// 1.�趨Ŀ��λ��
//	ServoB.target =   106;//rect_value[1]
//	// 2.��ȡ��ǰλ��
//	ServoB.now = red_value[1];
//	
//	// 3.PID�������������
//	pid_cal(&ServoB);
////	if(ServoA.out < 0)ServoA.out = -ServoA.out;
//	
//	// �������޷�
//	pidout_Servo_limit(&ServoB);
//	
//	// 4.PID�����ֵ ��������
//	
//	ServoB_Speed.target = ServoB.out;
//	
//	ServoB_Speed.now = redYSpeed;

//	pid_cal(&ServoB_Speed);
//	pidout_Servo_limit(&ServoB_Speed);
//	
//	if(ServoB_Speed.out < 0)ServoB_Speed.out = -ServoB_Speed.out;	
////	Servo_SetAngleB_2(ServoB_Speed.out);

//	//�������pid
////	Servo_SetAngleA_2(ServoA_Speed.out);
//	ServoB_Pid.target = ServoB_Speed.out;
//	
//	ServoB_Pid.now =ServoB_Pid.now;
//	
//	if(ServoB_Pid.now>=ServoB_Pid.target && ServoB_Pid.target - ServoB_Pid.now > 0)ServoB_Pid.now=ServoB_Pid.target;
//	if(ServoB_Pid.now<=0)ServoB_Pid.now=0;
//	
//	pid_cal(&ServoB_Pid);
//	
//	if(ServoB_Pid.out>0.1f)ServoB_Pid.out=0.1f;
//	else if(ServoB_Pid.out<-0.1f)ServoB_Pid.out=-0.1f;
//	
//	Servo_SetAngleB_2(ServoB_Pid.now);
//	
//	
//	redY_last = red_value[1];
//	
//}

void pid_control_Y(void)
{
	redYSpeed = red_value[0] - redY_last;
	
	// 1.�趨Ŀ��λ��
	ServoB.target =   116;//rect_value[1]
	// 2.��ȡ��ǰλ��
	ServoB.now = red_value[0];
	
	// 3.PID�������������
	pid_cal(&ServoB);
//	if(ServoA.out < 0)ServoA.out = -ServoA.out;
	
	// �������޷�
	pidout_Servo_limit(&ServoB);
	
	// 4.PID�����ֵ ��������
//	if(ServoB.target - ServoB.now <= 3 && ServoB.target - ServoB.now >= 0)ServoB.out =0;
//	if(ServoB.target - ServoB.now >= -3 && ServoB.target - ServoB.now <= 0)ServoB.out =0;
	
//	Servo_SetAngleB_2(ServoB.out);
	
	ServoB_Speed.target = ServoB.out;
	
	ServoB_Speed.now = redYSpeed;

	pid_cal(&ServoB_Speed);
	pidout_Servo_limit(&ServoB_Speed);
//	

//	Servo_SetAngleB_2(ServoB_Speed.out);

//	if(ServoB_Speed.out < 0)ServoB_Speed.out = -ServoB_Speed.out;	
////	Servo_SetAngleB_2(ServoB_Speed.out);

//	//�������pid
////	Servo_SetAngleA_2(ServoA_Speed.out);
	ServoB_Pid.target = ServoB_Speed.out;
//	
	ServoB_Pid.now =ServoB_Pid.now;
//	
	if(ServoB_Pid.now>=ServoB_Pid.target && ServoB_Speed.out >= 0)ServoB_Pid.now=ServoB_Pid.target;
	if(ServoB_Pid.now<=ServoB_Pid.target && ServoB_Speed.out < 0)ServoB_Pid.now=ServoB_Pid.target;
//	
	pid_cal(&ServoB_Pid);
//	
	if(ServoB_Pid.out>3)ServoB_Pid.out=3;
	else if(ServoB_Pid.out<-3)ServoB_Pid.out=-3;
//	
	Servo_SetAngleB_2(ServoB_Pid.now);
//	
//	
	redY_last = red_value[0];
//	
}
*/


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
		Emm_V5_Pos_Control(MOTOR_ADDR, 0, 1200, 0, (uint32_t)target_pulses, 1, 0);
	} else {
		Emm_V5_Pos_Control(MOTOR_ADDR, 1, 1200, 0, (uint32_t)(-target_pulses), 1, 0);
	}

	motor_command_pulses = target_pulses;
}

void pid_control__26Y(void)
{
	static float filtered_velocity = 0.0f;
	static float relative_y_last = 0.0f;
	static float target_y_last = 0.0f;
	static float breakaway_start_error = 0.0f;
	static uint8_t stuck_ticks = 0u;
	static bool breakaway_active = false;
	static bool control_initialized = false;

	Y = ball_error;  // 视觉小球绝对位置

	/* 在目标坐标系中控制：负值表示球在目标负侧，正值表示球在目标正侧。 */
	float target_y = ball_target_get();
	float relative_y = Y - target_y;
	bool target_changed = !control_initialized || target_y != target_y_last;

	/* 切换目标时丢弃旧的卡滞状态，避免沿旧目标方向施加起动倾角。 */
	if(target_changed) {
		stuck_ticks = 0u;
		breakaway_active = false;
		relative_y_last = relative_y;
		target_y_last = target_y;
		control_initialized = true;
	}

	float position_delta = relative_y - relative_y_last;
	redYSpeed = position_delta;  // 保留旧诊断变量，单位为每控制周期 cm

	/* PID 固定跟踪相对坐标系的零点，ball_target_y 可随时更改。 */
	pidY.target = 0.0f;
	pidY.now = relative_y;
	pid_cal(&pidY);

	/* 视觉速度存在帧间抖动和偶发零值，先低通滤波再参与制动。 */
	filtered_velocity += BALL_VELOCITY_FILTER_ALPHA * (ball_velocity - filtered_velocity);

	/* 仅在距目标超过阈值时使用起动倾角；进入近目标区域立即退出。 */
	if(breakaway_active) {
		if((relative_y <= STUCK_POSITION_THRESHOLD && relative_y >= -STUCK_POSITION_THRESHOLD) ||
		   (breakaway_start_error < 0.0f &&
		    relative_y >= breakaway_start_error + BREAKAWAY_RELEASE_DISTANCE) ||
		   (breakaway_start_error > 0.0f &&
		    relative_y <= breakaway_start_error - BREAKAWAY_RELEASE_DISTANCE)) {
			breakaway_active = false;
		}
		stuck_ticks = 0u;
	} else if((relative_y > STUCK_POSITION_THRESHOLD || relative_y < -STUCK_POSITION_THRESHOLD) &&
	          position_delta > -STUCK_POSITION_DELTA &&
	          position_delta < STUCK_POSITION_DELTA) {
		if(stuck_ticks < STUCK_TICKS_REQUIRED) {
			stuck_ticks++;
		}
		if(stuck_ticks >= STUCK_TICKS_REQUIRED) {
			breakaway_start_error = relative_y;
			breakaway_active = true;
		}
	} else {
		stuck_ticks = 0u;
	}

	/* 球向正方向运动时给负倾角刹车，反之亦然。 */
	pidY_velocity_damping = -VELOCITY_DAMPING_K * filtered_velocity;
	if(pidY_velocity_damping > VELOCITY_DAMPING_LIMIT) {
		pidY_velocity_damping = VELOCITY_DAMPING_LIMIT;
	} else if(pidY_velocity_damping < -VELOCITY_DAMPING_LIMIT) {
		pidY_velocity_damping = -VELOCITY_DAMPING_LIMIT;
	}

	// 位置环倾角叠加速度制动；卡滞时保持最小起动倾角。
	float cmd_angle = pidY.out + pidY_velocity_damping;
	if(breakaway_active) {
		if(breakaway_start_error < 0.0f && cmd_angle < BREAKAWAY_POS_ANGLE) {
			cmd_angle = BREAKAWAY_POS_ANGLE;
		} else if(breakaway_start_error > 0.0f && cmd_angle > BREAKAWAY_NEG_ANGLE) {
			cmd_angle = BREAKAWAY_NEG_ANGLE;
		}
	}

	// 负方向（下降）机构补偿 ×1.2。
	if(cmd_angle < 0.0f) {
		cmd_angle *= 1.2f;
	}
	motor_set_angle(cmd_angle);

	relative_y_last = relative_y;
	Y_last = Y;
}

void pidout_limit_Y(pid_t *pid)
{
	PIDOUT_CLAMP(pid, -50.0f, 50.0f);
}