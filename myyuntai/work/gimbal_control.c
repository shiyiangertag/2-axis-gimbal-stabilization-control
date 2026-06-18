#include "gimbal_control.h"
#include "pid.h"
#include "my_motor.h"
#include "stm32f4xx_hal.h" 
#include <stdio.h>
#include <math.h>

PID_Controller_t pid_x;
PID_Controller_t pid_y;

volatile uint32_t last_target_time = 0;
volatile bool is_tracking_active = false;
volatile uint8_t gimbal_run_enable = 0;

/* ======================================================== */
/* 全局可调参数变量                                         */
/* ======================================================== */
volatile float    gimbal_deadzone            = GIMBAL_DEADZONE_DEFAULT;
volatile uint32_t gimbal_timeout_ms          = GIMBAL_TIMEOUT_MS_DEFAULT;

volatile float    gimbal_kp_base             = PID_KP_BASE_DEFAULT;
volatile float    gimbal_ki_base             = PID_KI_BASE_DEFAULT;
volatile float    gimbal_kd_base             = PID_KD_BASE_DEFAULT;

volatile float    gimbal_err_threshold_small = ERR_THRESHOLD_SMALL_DEFAULT;

extern volatile int8_t Target_Speed_X;
extern volatile int8_t Target_Speed_Y;
extern volatile uint8_t New_Motor_Cmd_Flag;
extern volatile uint8_t New_Forward_Flag;

void Gimbal_Motor_Init(void)
{
    PID_Init(&pid_x, gimbal_kp_base, gimbal_ki_base, gimbal_kd_base, 100.0f, 30.0f);
    PID_Init(&pid_y, gimbal_kp_base, gimbal_ki_base, gimbal_kd_base, 100.0f, 30.0f);
    Motor_Init();
    gimbal_run_enable = 0;
    
    // 恢复所有默认设置数值
    gimbal_deadzone            = GIMBAL_DEADZONE_DEFAULT;
    gimbal_timeout_ms          = GIMBAL_TIMEOUT_MS_DEFAULT;
    gimbal_kp_base             = PID_KP_BASE_DEFAULT;
    gimbal_ki_base             = PID_KI_BASE_DEFAULT;
    gimbal_kd_base             = PID_KD_BASE_DEFAULT;
    gimbal_err_threshold_small = ERR_THRESHOLD_SMALL_DEFAULT;
}

/* ======================================================== */
/* 调参赋值与打印接口                                       */
/* ======================================================== */
void Gimbal_Set_Deadzone(float dz) { gimbal_deadzone = dz; printf("[Gimbal] Deadzone = %.2f\r\n", gimbal_deadzone); }
void Gimbal_Set_Timeout(uint32_t to) { gimbal_timeout_ms = to; printf("[Gimbal] Timeout = %u ms\r\n", gimbal_timeout_ms); }
void Gimbal_Set_Kp_Base(float kp) { gimbal_kp_base = kp; printf("[Gimbal] KP Base = %.3f\r\n", gimbal_kp_base); }
void Gimbal_Set_Ki_Base(float ki) { gimbal_ki_base = ki; printf("[Gimbal] KI Base = %.3f\r\n", gimbal_ki_base); }
void Gimbal_Set_Kd_Base(float kd) { gimbal_kd_base = kd; printf("[Gimbal] KD Base = %.3f\r\n", gimbal_kd_base); }
void Gimbal_Set_Err_Threshold_Small(float ths) { gimbal_err_threshold_small = ths; printf("[Gimbal] Small Err Threshold = %.1f\r\n", gimbal_err_threshold_small); }

/* ======================================================== */
/* 渐进式自适应 PID 核心算法 (步长微调 + 严格限幅)          */
/* ======================================================== */
void Gimbal_Adaptive_Params_Tune(PID_Controller_t *pid, float error)
{
    float abs_err = fabsf(error);
    float err_rate = fabsf(error - pid->last_error);

    /* 1. 根据偏差区间：决定 P 和 I 的渐进增减方向 */
    if (abs_err > ERR_THRESHOLD_LARGE)
    {
        pid->Kp += PID_KP_STEP;
        pid->Ki -= PID_KI_STEP;
    }
    else if (abs_err < gimbal_err_threshold_small)
    {
        pid->Kp -= PID_KP_STEP;
        pid->Ki += PID_KI_STEP;
    }
    else
    {
        if (pid->Kp > gimbal_kp_base)      pid->Kp -= PID_KP_STEP;
        else if (pid->Kp < gimbal_kp_base) pid->Kp += PID_KP_STEP;

        if (pid->Ki > gimbal_ki_base)      pid->Ki -= PID_KI_STEP;
        else if (pid->Ki < gimbal_ki_base) pid->Ki += PID_KI_STEP;
    }

    /* 2. 根据震荡变化率：决定 D 的渐进增减方向 */
    if (err_rate > ERR_RATE_THRESHOLD)
    {
        pid->Kd += PID_KD_STEP;
    }
    else
    {
        if (pid->Kd > gimbal_kd_base)      pid->Kd -= PID_KD_STEP;
        else if (pid->Kd < gimbal_kd_base) pid->Kd += PID_KD_STEP;
    }

    /* 3. 绝对安全限幅过滤 (非常重要) */
    if (pid->Kp > gimbal_kp_base + PID_KP_MAX_OFFSET) pid->Kp = gimbal_kp_base + PID_KP_MAX_OFFSET;
    if (pid->Kp < gimbal_kp_base - PID_KP_MAX_OFFSET) pid->Kp = gimbal_kp_base - PID_KP_MAX_OFFSET;
    if (pid->Kp < 0.0f) pid->Kp = 0.0f; 

    if (pid->Ki > gimbal_ki_base + PID_KI_MAX_OFFSET) pid->Ki = gimbal_ki_base + PID_KI_MAX_OFFSET;
    if (pid->Ki < 0.0f) pid->Ki = 0.0f; 

    if (pid->Kd > gimbal_kd_base + PID_KD_MAX_OFFSET) pid->Kd = gimbal_kd_base + PID_KD_MAX_OFFSET;
    if (pid->Kd < gimbal_kd_base - PID_KD_MAX_OFFSET) pid->Kd = gimbal_kd_base - PID_KD_MAX_OFFSET;
    if (pid->Kd < 0.0f) pid->Kd = 0.0f;
}

/* ======================================================== */
/* 控制中枢 (10ms 中断调用)                                 */
/* ======================================================== */
void Gimbal_Calculate_Control_Flow(float dx, float dy)
{
    if (gimbal_run_enable == 0)
    {
        Target_Speed_X = 0;
        Target_Speed_Y = 0;
        New_Motor_Cmd_Flag = 1;
        return;
    }

    last_target_time = HAL_GetTick();
    is_tracking_active = true;

    float out_x = 0.0f;
    float out_y = 0.0f;

    // X 轴：死区与积分分离检查
    if (dx > -gimbal_deadzone && dx < gimbal_deadzone) 
    {
        out_x = 0.0f;
        pid_x.integral = 0.0f; // 冻结死区积分，防止震荡
    }
    else 
    {
        Gimbal_Adaptive_Params_Tune(&pid_x, dx);
        out_x = PID_Compute(&pid_x, dx);
    }

    // Y 轴：死区与积分分离检查
    if (dy > -gimbal_deadzone && dy < gimbal_deadzone) 
    {
        out_y = 0.0f;
        pid_y.integral = 0.0f;
    }
    else 
    {
        Gimbal_Adaptive_Params_Tune(&pid_y, dy);
        out_y = PID_Compute(&pid_y, dy);
    }

    // 轴反向匹配
    if (REVERSE_X_AXIS) out_x = -out_x;
    if (REVERSE_Y_AXIS) out_y = -out_y;

    // 动作柔化：一阶低通滤波
    static float filtered_x = 0.0f;
    static float filtered_y = 0.0f;
    const float  alpha = 0.4f; // 滤波系数(越小越平滑)

    filtered_x = (1.0f - alpha) * filtered_x + alpha * out_x;
    filtered_y = (1.0f - alpha) * filtered_y + alpha * out_y;

    // 更新全局目标变量
    Target_Speed_X = (int8_t)filtered_x;
    Target_Speed_Y = (int8_t)filtered_y;
    
    New_Motor_Cmd_Flag = 1;
    New_Forward_Flag = 1;  
}

void Gimbal_Set_Run_State(uint8_t state)
{
    gimbal_run_enable = state;
    if (gimbal_run_enable == 0)
    {
        Target_Speed_X = 0;
        Target_Speed_Y = 0;
        New_Motor_Cmd_Flag = 1;
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        Motor_Stop();
        is_tracking_active = false;
        printf("[Gimbal State] STOP Active! PID and Motors forced to 0.\r\n");
    }
    else
    {
        printf("[Gimbal State] START Active! PID closed-loop control enabled.\r\n");
    }
}

void Gimbal_Reset_Tracking(void)
{
    if (is_tracking_active) {
        Motor_Stop();
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        is_tracking_active = false;
        printf("[Gimbal Warning] Target Lost! Reset Flow.\r\n");
    }
}

void Gimbal_Watchdog_Task(void)
{
    if (gimbal_run_enable && is_tracking_active && ((HAL_GetTick() - last_target_time) > gimbal_timeout_ms)) {
        Gimbal_Reset_Tracking();
    }
}