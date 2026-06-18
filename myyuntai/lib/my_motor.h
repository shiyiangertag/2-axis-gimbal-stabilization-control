#ifndef __MY_MOTOR_H_
#define __MY_MOTOR_H_

#include "usart.h"       // 提供 huart2, huart4 声明
#include "Emm_V5.h"      // 提供步进电机底层协议
#include <stdint.h>
#include <stdbool.h>

/* 电机控制宏定义 */
#define MOTOR_X_ADDR        0x01          // X轴电机地址
#define MOTOR_Y_ADDR        0x01          // Y轴电机地址
#define MOTOR_X_UART        huart2        // X轴电机串口 (左)
#define MOTOR_Y_UART        huart4        // Y轴电机串口 (右)

// 【关键修改】将原本的 3 RPM 改为 100 RPM，释放云台速度潜能
#define MOTOR_MAX_SPEED     100           
#define MOTOR_ACCEL         0             // 电机加速度(0表示直接启动)
#define MOTOR_SYNC_FLAG     false         // 电机同步标志

/* 函数声明 */
void Motor_Init(void);                                     // 电机初始化
void Motor_Set_Speed(int8_t x_percent, int8_t y_percent);  // 设置XY电机速度(百分比)
void Motor_Stop(void);                                     // 停止所有电机

#endif /* __MY_MOTOR_H_ */
