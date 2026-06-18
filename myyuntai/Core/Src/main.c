/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myprintf.h" // 阻塞打印
#include "myuart.h"   // DMA收发模块
#include "gimbal_control.h"
#include "Emm_V5.h"
#include "my_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// 引入跨文件信使变量与标志位
extern volatile uint8_t visual_data_ready;
extern volatile int visual_dx;
extern volatile int visual_dy;

extern uint8_t visual_raw_buf[];
extern volatile uint16_t visual_raw_len;

// 引入 PID 计算完毕后的全局目标速度变量
volatile int8_t Target_Speed_X = 0;
volatile int8_t Target_Speed_Y = 0;
volatile uint8_t New_Motor_Cmd_Flag = 0; // 电机指令更新标志
volatile uint8_t New_Forward_Flag = 0;   // 串口转发通知标志

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_Delay(1000);
	Motor_Stop();
  printf("\r\n===================================\r\n");
  printf("  Gimbal System Initialized Success  \r\n"); 
  printf("===================================\r\n");

  MyUART_Init();         // 启动 DMA 接收池与中断拦截
  Gimbal_Motor_Init();   // 初始化云台 PID 与电机参数

  // 核心：启动 TIM1 控制定时中断 (10ms 节拍)
  HAL_TIM_Base_Start_IT(&htim1); 
  
  // 缓存上一次下发的电机速度，实现“变化才发送”防轰炸机制
  int8_t last_speed_x = -128;
  int8_t last_speed_y = -128;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */
		// 任务一：电机调控调度的四肢执行
      if (New_Motor_Cmd_Flag == 1)
      {
          New_Motor_Cmd_Flag = 0; // 消费标志
          
          // 【核心防轰炸】：只有当前后计算转速不同时，才真正向串口写硬件指令
          if (Target_Speed_X != last_speed_x || Target_Speed_Y != last_speed_y)
          {
              Motor_Set_Speed(Target_Speed_X, Target_Speed_Y);
              last_speed_x = Target_Speed_X;
              last_speed_y = Target_Speed_Y;
          }
      }

      // 任务二：串口1调试数据高级转发 (非阻塞，绝不卡死 CPU)
      if (New_Forward_Flag == 1)
      {
          New_Forward_Flag = 0; // 消费标志
          
          if (visual_raw_len > 0 && huart1.gState == HAL_UART_STATE_READY)
          {
              HAL_UART_Transmit_IT(&huart1, visual_raw_buf, visual_raw_len);
          }
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief 定时器溢出中断回调函数 (TIM1 触发，每 10ms 进一次)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) 
    {
        // 1. 安全防线：检查摄像头是否断联超时
        Gimbal_Watchdog_Task();

        // 2. 控制大脑：一旦后台解好包新的坐标数据
        if (visual_data_ready == 1)
        {
            visual_data_ready = 0; // 消费就绪信号

            // 计算 PID，直接调控速度，将结果写入全局缓冲中
            Gimbal_Calculate_Control_Flow((float)visual_dx, (float)visual_dy);
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
