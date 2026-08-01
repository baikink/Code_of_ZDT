#include "pid.h"
#include "Emm_V5.h"              /* ball_error、Emm_V5_Pos_Control */

extern float ball_velocity;  /* 视觉小球速度，定义在 main.c */
extern float motor_current_angle;  /* 电机实时角度(°)，基于上电零点 */

#define PULSES_PER_DEG  17.78f  /* 32细分，1°=17.78脉冲 */

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

pid_t pidY;
pid_t pidY_Speed;

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
	// ����޷�
	if(pid->out>=MAX_DUTY)
		pid->out=MAX_DUTY;
	if(pid->out<=0)
		pid->out=0;
}

void pidout_Servo_limit(pid_t *pid)
{
	// ����޷�
	if(pid->out>=40)
		pid->out=40;
	if(pid->out<=-40)
		pid->out=-40;
}





///////////////////////////////////////26///////////////////////
void pid_control__26Y(void)
{
	Y = ball_error;              // 视觉小球位置
	redYSpeed = ball_velocity;   // 视觉小球速度（不自己算差分）
		// 1.设置目标位置
	pidY.target =   0;
		// 2.获取当前位置
	pidY.now = Y;
		// 3.PID控制器计算输出
	pid_cal(&pidY);
		// 4.限制输出范围
	pidout_limit_Y(&pidY);

	// ═══ 绝对角度限幅：基于上电零点的 ±300 脉冲范围 ═══
	float cur_angle = motor_current_angle;              // 电机当前绝对角度(°)
	float cur_pulses = cur_angle * PULSES_PER_DEG;      // 换算脉冲
	float target_pulses = cur_pulses + pidY.out;         // 如果执行本次增量后的绝对位置
	if(target_pulses >  300.0f) pidY.out =  300.0f - cur_pulses;  // 截断：不能超正向
	if(target_pulses < -300.0f) pidY.out = -300.0f - cur_pulses;  // 截断：不能超反向
	if(pidY.out >  300.0f) pidY.out =  300.0f;          // 防溢出
	if(pidY.out < -300.0f) pidY.out = -300.0f;

	// 电机执行：直接使用位置环输出
	uint32_t clk = (uint32_t)(pidY.out > 0 ? pidY.out : -pidY.out);
	if(pidY.out>0)Emm_V5_Pos_Control(MOTOR_ADDR, 0, 10, 0, clk, 0, 0);
	else if(pidY.out<0)Emm_V5_Pos_Control(MOTOR_ADDR, 1, 10, 0, clk, 0, 0);
	else Emm_V5_Pos_Control(MOTOR_ADDR, 0, 10, 0, 0, 0, 0);

	Y_last = Y;
}


void pidout_limit_Y(pid_t *pid)
{
	// 涓ら潰闄愬箙
	if(pid->out>=300)
		pid->out=300;
	if(pid->out<=-300)
		pid->out=-300;
}