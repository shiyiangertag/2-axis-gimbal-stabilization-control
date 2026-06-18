#ifndef __MYUART_H
#define __MYUART_H

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#define CAMERA_RX_SIZE  512

void MyUART_Init(void);
void MyUART_USART6_IDLE_Handler(void);

#endif /* __MYUART_H */