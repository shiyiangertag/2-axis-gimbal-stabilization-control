#include "pid.h"

void PID_Init(PID_Controller_t *pid, float p, float i, float d, float max_out, float max_integral) 
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->max_out = max_out;
    pid->max_integral = max_integral;
    PID_Reset(pid);
}

float PID_Compute(PID_Controller_t *pid, float current_error) 
{
    pid->error = current_error;
    
    // 积分累加与抗饱和限幅
    pid->integral += pid->error;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    
    // PID 计算
    float p_out = pid->Kp * pid->error;
    float i_out = pid->Ki * pid->integral;
    float d_out = pid->Kd * (pid->error - pid->last_error);
    float total_out = p_out + i_out + d_out;
    
    // 输出限幅
    if (total_out > pid->max_out) total_out = pid->max_out;
    else if (total_out < -pid->max_out) total_out = -pid->max_out;
    
    // 更新历史误差
    pid->last_error = pid->error;
    
    return total_out;
}

void PID_Reset(PID_Controller_t *pid) 
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
}