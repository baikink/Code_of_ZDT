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
float pidY_filtered_velocity = 0.0f;  /* 低通滤波后的视觉速度 (cm/s)，供 VOFA ch5 */

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
	static float breakaway_start_error = 0.0f;
	static float breakaway_angle = 0.0f;
	static uint8_t stuck_ticks = 0u;
	static uint8_t breakaway_ramp_ticks = 0u;
	static bool breakaway_active = false;
	static bool control_initialized = false;
	static bool target_hold_active = false;

	Y = ball_error;  // 视觉小球绝对位置

	/* 在目标坐标系中控制：负值表示球在目标负侧，正值表示球在目标正侧。 */
	float target_y = ball_target_get();
	float relative_y = Y - target_y;
	bool target_changed = !control_initialized || target_y != target_y_last;

	/* 切换目标时丢弃旧的卡滞状态，避免沿旧目标方向施加起动倾角。 */
	if(target_changed) {
		stuck_ticks = 0u;
		breakaway_ramp_ticks = 0u;
		breakaway_active = false;
		breakaway_angle = 0.0f;
		target_hold_active = false;
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

	/* 接近目标后的到位保持。
	 *
	 * 这里不是简单的“位置一小就停”，而是同时满足：
	 *   1) |位置误差| <= 0.4cm：比 ±1cm 的验收线明显更紧，真正贴近目标；
	 *   2) |速度| <= 0.4cm/s：球确实已经慢下来，而不是高速穿越目标点。
	 *
	 * 一旦进入保持态，就直接撤掉控制输出，让平台回到 0°；
	 * 只有当误差重新长到 0.6cm 以上时才恢复闭环。这个回差能避免视觉噪声、
	 * 步进脉冲量化和轻微回弹把系统卡在“进死区/出死区”的反复切换里。
	 */
	if(target_hold_active) {
		if(abs_relative_y >= TARGET_HOLD_POSITION_EXIT) {
			target_hold_active = false;
		}
	} else if(abs_relative_y <= TARGET_HOLD_POSITION_ENTER &&
	          abs_estimated_velocity <= TARGET_HOLD_SPEED_LIMIT) {
		target_hold_active = true;
	}

	if(target_hold_active) {
		/* 进入近目标稳态区后，不退出闭环，也不回零/锁角。
		 * 做法是：仅禁止 breakaway，且稍后把外环目标速度压到 0，
		 * 让内环继续按“0 速度”闭环保持球静止。
		 * 这样既保留了微调能力，又避免平台在目标附近追来追去。
		 */
		stuck_ticks = 0u;
		breakaway_ramp_ticks = 0u;
		breakaway_active = false;
		breakaway_angle = 0.0f;
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
	if(target_hold_active) {
		/* 近目标时外环不再继续命令小球移动，而是把目标速度压到 0。
		 * 内环仍保持闭环，用很小的倾角持续抑制残余滚动，帮助球真正停在目标附近。
		 */
		pidY.out = 0.0f;
	}

	/* 先限制本地估计速度，再低通滤波，避免位置量化造成的单帧尖峰把速度环推入饱和。
	 * VOFA ch2 继续保留 MaixCAM 原始速度，仅用于诊断比较，不再直接参与控制。
	 */
	float control_velocity = estimated_velocity;
	if(control_velocity > BALL_VELOCITY_CONTROL_LIMIT) {
		control_velocity = BALL_VELOCITY_CONTROL_LIMIT;
	} else if(control_velocity < -BALL_VELOCITY_CONTROL_LIMIT) {
		control_velocity = -BALL_VELOCITY_CONTROL_LIMIT;
	}
	pidY_filtered_velocity += BALL_VELOCITY_FILTER_ALPHA *
	                         (control_velocity - pidY_filtered_velocity);

	/* 仅在距目标超过阈值时使用起动倾角；进入近目标区域立即退出。
	 *
	 * breakaway 现改为“分级加力”：
	 *   - 第一次检测到卡滞时，只给 BREAKAWAY_*_ANGLE_BASE；
	 *   - 若后续每过 BREAKAWAY_RAMP_TICKS 仍几乎不动，就把起动角再加一级；
	 *   - 一直加到 BREAKAWAY_*_ANGLE_MAX 为止。
	 *
	 * 这样能避免固定 -3.6° 在某些位置推不动球时永久卡死。
	 */
	if(breakaway_active) {
		if((relative_y <= STUCK_POSITION_THRESHOLD && relative_y >= -STUCK_POSITION_THRESHOLD) ||
		   (breakaway_start_error < 0.0f &&
		    relative_y >= breakaway_start_error + BREAKAWAY_RELEASE_DISTANCE) ||
		   (breakaway_start_error > 0.0f &&
		    relative_y <= breakaway_start_error - BREAKAWAY_RELEASE_DISTANCE)) {
			breakaway_active = false;
			breakaway_angle = 0.0f;
			breakaway_ramp_ticks = 0u;
		} else if(position_delta > -STUCK_POSITION_DELTA &&
		          position_delta < STUCK_POSITION_DELTA) {
			if(breakaway_ramp_ticks < BREAKAWAY_RAMP_TICKS) {
				breakaway_ramp_ticks++;
			} else {
				breakaway_ramp_ticks = 0u;
				if(breakaway_start_error < 0.0f) {
					breakaway_angle += BREAKAWAY_ANGLE_STEP;
					if(breakaway_angle > BREAKAWAY_POS_ANGLE_MAX) {
						breakaway_angle = BREAKAWAY_POS_ANGLE_MAX;
					}
				} else {
					breakaway_angle -= BREAKAWAY_ANGLE_STEP;
					if(breakaway_angle < BREAKAWAY_NEG_ANGLE_MAX) {
						breakaway_angle = BREAKAWAY_NEG_ANGLE_MAX;
					}
				}
			}
		} else {
			breakaway_ramp_ticks = 0u;
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
			breakaway_ramp_ticks = 0u;
			breakaway_angle = (relative_y < 0.0f) ? BREAKAWAY_POS_ANGLE_BASE : BREAKAWAY_NEG_ANGLE_BASE;
		}
	} else {
		stuck_ticks = 0u;
	}

	/* 外环给出小球目标速度，内环以本地估计速度为反馈输出平台倾角。
	 *
	 * 内环积分是消除稳态位置误差的关键：命令角 0° 只是上电机械零点，
	 * 不等于力学水平角。两者之差加上滚动摩擦是一个常值扰动，
	 * 纯 P 串级无法消除，小球只会停在“实际水平”的位置而不是目标位置。
	 * 内环积分会持续调整倾角直到实测速度跟上目标速度，
	 * 静止平衡时即自动找到真实水平角，外环误差随之收敛到 0。
	 */
	pidY_Speed.target = pidY.out;
	pidY_Speed.now = pidY_filtered_velocity;

	/* 起动补偿期间倾角由 BREAKAWAY_* 强制接管，实际执行值与内环输出无关，
	 * 此时继续积分只会累积无效误差，退出后造成一次反向过冲，故冻结积分。 */
	float iout_frozen = pidY_Speed.iout;
	pid_cal_clamped(&pidY_Speed, -SPEED_LOOP_ANGLE_LIMIT, SPEED_LOOP_ANGLE_LIMIT,
	                SPEED_LOOP_I_LIMIT);
	if(breakaway_active) {
		pidY_Speed.iout = iout_frozen;
		pidY_Speed.out = pidY_Speed.pout + pidY_Speed.iout + pidY_Speed.dout;
		if(pidY_Speed.out > SPEED_LOOP_ANGLE_LIMIT) {
			pidY_Speed.out = SPEED_LOOP_ANGLE_LIMIT;
		} else if(pidY_Speed.out < -SPEED_LOOP_ANGLE_LIMIT) {
			pidY_Speed.out = -SPEED_LOOP_ANGLE_LIMIT;
		}
	}

	pidY_velocity_damping = pidY_Speed.out;  // 保留旧变量名，供 VOFA 观察限幅后的速度环输出

	// 速度环输出作为倾角命令；卡滞时由分级加力的 breakaway 角接管。
	float cmd_angle = pidY_velocity_damping;
	if(breakaway_active) {
		if(breakaway_start_error < 0.0f && cmd_angle < breakaway_angle) {
			cmd_angle = breakaway_angle;
		} else if(breakaway_start_error > 0.0f && cmd_angle > breakaway_angle) {
			cmd_angle = breakaway_angle;
		}
	}

	// 负方向（下降）机构补偿 ×1.2。
	if(cmd_angle < 0.0f) {
		cmd_angle *= 1.2f;
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