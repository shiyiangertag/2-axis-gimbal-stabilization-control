#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#define REVERSE_X_AXIS      1       // 遇反向置1，默认为0
#define REVERSE_Y_AXIS      1       // 遇反向置1，默认为0

/* ======================================================== */
/* 默认初始值宏定义 (上电默认值)                            */
/* ======================================================== */
#define GIMBAL_DEADZONE_DEFAULT     5.0f   // 默认视觉死区(像素)
#define GIMBAL_TIMEOUT_MS_DEFAULT   50     // 默认断联超时停机阈值(毫秒)

#define PID_KP_BASE_DEFAULT         0.15f  // 比例项默认基准值
#define PID_KI_BASE_DEFAULT         0.05f  // 积分项默认基准值
#define PID_KD_BASE_DEFAULT         0.12f  // 微分项默认基准值

#define ERR_THRESHOLD_SMALL_DEFAULT 30.0f  // 小偏差区域判定阈值(像素)
#define ERR_THRESHOLD_LARGE         80.0f  // 大偏差区域判定阈值(固定)
#define ERR_RATE_THRESHOLD          15.0f  // 偏差变化率剧烈阈值(固定)

/* ======================================================== */
/* 渐进式自适应 PID 微调步长与限幅宏                        */
/* ======================================================== */
// 1. 每次(10ms)累加或递减的极小步长
#define PID_KP_STEP         0.002f  
#define PID_KI_STEP         0.001f  
#define PID_KD_STEP         0.002f  

// 2. 允许偏离基准值的最大安全幅度绝对值 (防止参数无限涨跌)
#define PID_KP_MAX_OFFSET   0.10f   
#define PID_KI_MAX_OFFSET   0.05f   
#define PID_KD_MAX_OFFSET   0.10f   

/* 函数声明 */
void Gimbal_Motor_Init(void);
void Gimbal_Calculate_Control_Flow(float dx, float dy);
void Gimbal_Reset_Tracking(void);
void Gimbal_Watchdog_Task(void);
void Gimbal_Set_Run_State(uint8_t state);

/* 在线调参接口声明 */
void Gimbal_Set_Deadzone(float dz);
void Gimbal_Set_Timeout(uint32_t to);
void Gimbal_Set_Kp_Base(float kp);
void Gimbal_Set_Ki_Base(float ki);
void Gimbal_Set_Kd_Base(float kd);
void Gimbal_Set_Err_Threshold_Small(float ths);

#endif