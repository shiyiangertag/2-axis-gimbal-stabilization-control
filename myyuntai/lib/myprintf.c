#include "myprintf.h"

/* * 引入 USART1 的 HAL 库句柄。
 * 这个句柄通常由 STM32CubeMX 在 usart.c 或 main.c 中自动生成并定义。
 * 使用 extern 关键字可以直接在这里调用它。
 */
extern UART_HandleTypeDef huart1;

/* 告知 ARM 编译器不要使用半主机模式 (Semihosting) */
#pragma import(__use_no_semihosting)             

/* 标准库需要的底层文件结构体 */
struct __FILE 
{ 
    int handle; 
}; 

FILE __stdout;       

/* * 定义 _sys_exit() 以避免程序因为缺少该函数而卡死在半主机模式。
 */
void _sys_exit(int x) 
{ 
    x = x; 
} 

/* * 重定义 fputc 函数。
 * C 标准库的 printf 最终会调用此函数来逐个字符输出。
 */
int fputc(int ch, FILE *f)
{
    /* * 将字符通过 USART1 发送。
     * 参数解释：串口句柄, 数据指针, 数据长度(1字节), 超时时间(1000ms)
     */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
    return ch;
}
