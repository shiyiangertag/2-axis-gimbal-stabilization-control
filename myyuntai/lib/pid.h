#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* PID 控制器结构体 */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分累积量
    
    float max_out;      // 输出限幅
    float max_integral; // 积分限幅 (防积分饱和)
} PID_Controller_t;

/* 接口函数声明 */
void PID_Init(PID_Controller_t *pid, float p, float i, float d, float max_out, float max_integral);
float PID_Compute(PID_Controller_t *pid, float current_error);
void PID_Reset(PID_Controller_t *pid);

#endif /* __PID_H */