#include "myuart.h"
#include "gimbal_control.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

volatile uint8_t visual_data_ready = 0;
volatile int visual_dx = 0;            
volatile int visual_dy = 0;            

uint8_t visual_raw_buf[128];           
volatile uint16_t visual_raw_len = 0;  

uint8_t Camera_RxBuffer[CAMERA_RX_SIZE];

void MyUART_USART6_IDLE_Handler(void)
{
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        HAL_UART_DMAStop(&huart6);
        uint32_t data_len = CAMERA_RX_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        
        if (data_len > 0 && data_len < 128)
        {
            Camera_RxBuffer[data_len] = '\0';
            
            int temp_x, temp_y;
            if (sscanf((char*)Camera_RxBuffer, "R,%d,%d", &temp_x, &temp_y) == 2)
            {
                visual_dx = temp_x;
                visual_dy = temp_y;
                
                memcpy(visual_raw_buf, Camera_RxBuffer, data_len);
                visual_raw_len = data_len;
                
                visual_data_ready = 1;
            }
        }
        
        memset(Camera_RxBuffer, 0, CAMERA_RX_SIZE);
        HAL_UART_Receive_DMA(&huart6, Camera_RxBuffer, CAMERA_RX_SIZE);
    }
}

uint8_t u1_rx_byte;      
uint8_t cmd_buf[64];     
uint8_t cmd_idx = 0;     
uint8_t is_receiving = 0;

/* ======================================================== */
/* 串口1指令路由：解析调参指令                              */
/* ======================================================== */
void MyUART_ExecuteCommand(char *cmd)
{
    printf("[PC Cmd Ack] Executing: %s\r\n", cmd);
    
    // 1. 系统控制指令
    if (strcmp(cmd, "STOP") == 0)
    {
        Gimbal_Set_Run_State(0);
    } 
    else if (strcmp(cmd, "START") == 0)
    {
        Gimbal_Set_Run_State(1);
    }
    // 2. 基础配置指令
    else if (strncmp(cmd, "DZ=", 3) == 0)
    {
        float val;
        if (sscanf(cmd + 3, "%f", &val) == 1) Gimbal_Set_Deadzone(val);
    }
    else if (strncmp(cmd, "TO=", 3) == 0)
    {
        uint32_t val;
        if (sscanf(cmd + 3, "%u", &val) == 1) Gimbal_Set_Timeout(val);
    }
    // 3. PID 基础参数指令
    else if (strncmp(cmd, "KP=", 3) == 0)
    {
        float val;
        if (sscanf(cmd + 3, "%f", &val) == 1) Gimbal_Set_Kp_Base(val);
    }
    else if (strncmp(cmd, "KI=", 3) == 0)
    {
        float val;
        if (sscanf(cmd + 3, "%f", &val) == 1) Gimbal_Set_Ki_Base(val);
    }
    else if (strncmp(cmd, "KD=", 3) == 0)
    {
        float val;
        if (sscanf(cmd + 3, "%f", &val) == 1) Gimbal_Set_Kd_Base(val);
    }
    // 4. 小偏差区专属调参指令
    else if (strncmp(cmd, "THS=", 4) == 0)
    {
        float val;
        if (sscanf(cmd + 4, "%f", &val) == 1) Gimbal_Set_Err_Threshold_Small(val);
    }
    /* 注意：废弃了 KPS 和 KIS 的指令，因为改为了步长微调算法 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (u1_rx_byte == '$') {
            is_receiving = 1;
            cmd_idx = 0;
        } else if (u1_rx_byte == '#' && is_receiving) {
            cmd_buf[cmd_idx] = '\0';
            MyUART_ExecuteCommand((char*)cmd_buf);
            is_receiving = 0;
        } else if (is_receiving) {
            if (cmd_idx < 63) cmd_buf[cmd_idx++] = u1_rx_byte;
            else is_receiving = 0;
        }
        HAL_UART_Receive_IT(&huart1, &u1_rx_byte, 1);
    }
}

void MyUART_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &u1_rx_byte, 1);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6, Camera_RxBuffer, CAMERA_RX_SIZE);
}