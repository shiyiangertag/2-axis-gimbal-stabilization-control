#include "my_motor.h"

void Motor_Init(void)
{
    Emm_V5_En_Control(&MOTOR_X_UART, MOTOR_X_ADDR, true, MOTOR_SYNC_FLAG);
    Emm_V5_En_Control(&MOTOR_Y_UART, MOTOR_Y_ADDR, true, MOTOR_SYNC_FLAG);
    Motor_Stop();
}

void Motor_Set_Speed(int8_t x_percent, int8_t y_percent)
{
    uint8_t x_dir, y_dir;
    uint16_t x_speed, y_speed;

    /* 限制百分比范围 */
    if (x_percent > 100) x_percent = 100;
    if (x_percent < -100) x_percent = -100;
    if (y_percent > 100) y_percent = 100;
    if (y_percent < -100) y_percent = -100;

    /* 设置X轴方向 */
    if (x_percent >= 0) {
        x_dir = 0; 
    } else {
        x_dir = 1;              
        x_percent = -x_percent; 
    }

    /* 设置Y轴方向 */
    if (y_percent >= 0) {
        y_dir = 0; 
    } else {
        y_dir = 1;              
        y_percent = -y_percent; 
    }
    
    /* 计算实际速度值(百分比转换为RPM) */
    x_speed = (uint16_t)((x_percent * MOTOR_MAX_SPEED) / 100);
    y_speed = (uint16_t)((y_percent * MOTOR_MAX_SPEED) / 100);
    
    /* 控制XY轴电机 */
    Emm_V5_Vel_Control(&MOTOR_X_UART, MOTOR_X_ADDR, x_dir, x_speed, MOTOR_ACCEL, MOTOR_SYNC_FLAG);
    Emm_V5_Vel_Control(&MOTOR_Y_UART, MOTOR_Y_ADDR, y_dir, y_speed, MOTOR_ACCEL, MOTOR_SYNC_FLAG);
}

void Motor_Stop(void)
{
    Emm_V5_Stop_Now(&MOTOR_X_UART, MOTOR_X_ADDR, MOTOR_SYNC_FLAG);
    Emm_V5_Stop_Now(&MOTOR_Y_UART, MOTOR_Y_ADDR, MOTOR_SYNC_FLAG);
}
