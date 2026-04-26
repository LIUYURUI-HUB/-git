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
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gom_protocol.h"
#include <string.h>
#include "motor_controller.h"
#include "rs485.h"
#include <stdio.h>
#include "dm_driver.h"
#include "3508_driver.h"
#include <math.h>
#include <string.h>
#include "leg_kinematics.h"
#include "arm.h"
#include "arm_g.h"
#include "arm_task.h"
#include "Attitude_solution.h"
#include "../../Application/gait.h"
#include "AS01.h"
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct {
    float x;
    float y;
    float z;
} arm_target = {0.18f, 0.0f, -0.22f};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Currentpos currentpos;
LegAngles angles;
QuadrupedGait gait;

Arm_Lengths_t arm_params = {0.00f, 0.30f, 0.32f,0.084f};
Joint_Angles_t current_angles = {0};//全员初始化为 0
End_Pos_t end_effector_pos = {0};
uint32_t last_tick = 0;

extern ArmGravityParams_t arm_g_params;
extern Motor_3508_T Motors[4];
extern DM_Motor_t Arm_Motors[4];

MotorController ctrl1;
MotorController ctrl2;
__attribute__((section(".RAM_D1")))__attribute__((section(".RAM_D1")))MotorController ctrl2;
RemoteData_t myPacket;
uint8_t rx_buffer[32];
uint8_t rx_data[32];

//BMI088
float system_run_time = 0.0f;

float gyro[3] = {0};
float accel[3] = {0};
float temp = 0.0f;

uint32_t attitude_last_time = 0;
float debug_pitch = 0, debug_roll = 0, debug_yaw = 0;
int is_calibrated=1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void DM_Init(void);
void run_arm_kinematics(void);
void Task_Arm_Control(void);
void Task_Arm_Control_Line(void);
void Task_Arm_Control_Line_x(void);
extern void run_arm_drag_teach_mode(void);
extern uint8_t BMI088_init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


extern UART_HandleTypeDef huart7; //用于机械臂调参的串口


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM4) {
		static uint8_t step = 0;
		switch (step) {
		case 0:
			MotorController_SendCommand2(&ctrl2, 1);
			step = 1;
			break;
		case 1:
			MotorController_SendCommand2(&ctrl2, 2);
			step = 2;
			break;
		case 2:
			MotorController_SendCommand2(&ctrl2, 7);
			step = 3;
			break;
		case 3:
			MotorController_SendCommand2(&ctrl2, 8);
			step = 0;
			break;
		}
	}
	if (htim->Instance == TIM3) {
		static uint8_t time = 0;
		switch (time) {
		case 0:
			MotorController_SendCommand1(&ctrl1, 3);
			time = 1;
			break;
		case 1:
			MotorController_SendCommand1(&ctrl1, 4);
			time = 2;
			break;
		case 2:
			MotorController_SendCommand1(&ctrl1, 6);
			time = 3;
			break;
		case 3:
			MotorController_SendCommand1(&ctrl1, 5);
			time = 0;
			break;
		}
	}

}


/**
 * @brief   接收回调函数
 * @param   hfdcan      FDCAN句柄指针
 * @param   RxFifo0ITs  FIFO0中断标志
 * @retval  none
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        // 必须使用 while 循环将 FIFO 彻底读空，防止多电机并发导致丢帧
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

                // FDCAN1：接收 3508 的电机 1、2
                if (hfdcan->Instance == FDCAN1) {
                    D3508_Decode(RxData, (uint16_t)RxHeader.Identifier);
                }
                // FDCAN2：接收达妙电机
                else if (hfdcan->Instance == FDCAN2) {
                    DM_RX_Decode(RxData, (uint16_t)RxHeader.Identifier);
                }
                // FDCAN3：接收 3508 的电机 3、4
                else if (hfdcan->Instance == FDCAN3) {
                    D3508_Decode(RxData, (uint16_t)RxHeader.Identifier);
                }
            }
        }
    }
}

/**
 * @brief 机械臂控制任务：处理逆解与电机控制指令
 */
void Task_Arm_Control(void) {

    run_arm_to_pos(arm_target.x, arm_target.y, arm_target.z);
}



/**
 * @brief  系统状态更新任务（整合了姿态更新和运行时间计算）
 */
void Task_System_State_Update(void) {
    /* ========== 1. 姿态解算更新 (100Hz) ========== */
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - attitude_last_time) / 1000.0f;
    attitude_last_time = current_time;

    /* 限制 dt 范围 */
    if (dt > 0.1f) dt = 0.01f;
    if (dt < 0.001f) dt = 0.01f;

    /* 更新姿态 */
    Attitude_Update(dt);

    /* 获取解算角度 (用于调试或控制) */
    debug_pitch = g_Attitude.Angle_Pitch;
    debug_roll  = g_Attitude.Angle_Roll;
    debug_yaw   = g_Attitude.Angle_Yaw;

    /* ========== 2. 原有 BMI088 数据读取 (兼容旧代码) ========== */
    gyro[0] = g_Attitude.Gyro_X;
    gyro[1] = g_Attitude.Gyro_Y;
    gyro[2] = g_Attitude.Gyro_Z;
    accel[0] = g_Attitude.Accel_X;
    accel[1] = g_Attitude.Accel_Y;
    accel[2] = g_Attitude.Accel_Z;
    temp = g_Attitude.Temperature;

    /* ========== 3. 系统运行时间更新 ========== */
    uint32_t now = HAL_GetTick();
    if(now - last_tick >= 5) {
        float dt_sys = (now - last_tick) / 1000.0f;
        last_tick = now;
        system_run_time += dt_sys;


    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_UART7_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  FDCAN2_Filter_Init();
  FDCAN1_Filter_Init();
  FDCAN3_Filter_Init();
  HAL_Delay(1000);
  D3508_Init();
  DM_Init();

  Arm_Gravity_InitParams(&arm_g_params);
  Attitude_Init();

   MotorController_Init(&ctrl1, &huart2, RS485_REDE1_GPIO_Port, RS485_REDE1_Pin);
   MotorController_Init(&ctrl2, &huart3, RS485_REDE2_GPIO_Port, RS485_REDE2_Pin);
   uint32_t startTime=HAL_GetTick()/1000;
   HAL_TIM_Base_Start_IT(&htim3);
   HAL_Delay(50);
 //          for (uint8_t i = 0; i < 4; i++) {
 //              calibrate_leg_base_position(&gait, i, &ctrl2);
 //          }
 //          start_quadruped_gait(&gait,startTime);

   // last_kinematics_tick 供运动学独立时间调度
   uint32_t last_tick = HAL_GetTick();
   uint32_t last_kinematics_tick = HAL_GetTick();
   HAL_Delay(500);
   if(NRF24L01_Check_SPI() == 0) {
             NRF24L01_Init();
          }
   HAL_Delay(20);
  /* 【新增】姿态解算模块初始化 */


    /* 【新增】零偏校准 (保持传感器静止 2 秒) */
   if (g_Attitude.Init_OK) {
        Attitude_Calibrate_ZeroBias();
    }

  // 向达妙电机发送设置 0 位指令。
  for(uint16_t id = 0x01; id <= 0x04; id++) {
    DM_Set_Zero_Pos(id);
    HAL_Delay(10); // 必须加延时，等待指令发送完成及电机内部保存
  }

  for(uint16_t id = 0x01; id <= 0x04; id++) {
    DM_Motor_Enable(id);
    HAL_Delay(10); // 顺次启动，防止冲击电流
  }

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  last_tick = HAL_GetTick();
  attitude_last_time = HAL_GetTick();
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  uint32_t now = HAL_GetTick();

	  	    if(now - last_tick >= 5)
	  	    {
	            float dt = (now - last_tick) / 1000.0f;
	            last_tick = now;
	            system_run_time += dt;
//	            //计算 PID
//        	   for(int i = 0; i < 4; i++) {
//			   float motor_angle=200.0f;
//	           Motors[i].target_angle = (int64_t)(motor_angle * 19.0f * 8192.0f / 360.0f);
//	           PID_Calc_Position(i,Motors[i].target_angle);
//	            							   // PID_Calc_Speed is called inside PID_Calc_Position, no need to call again
//			   }
//        	   send_current();
//
//
//	            	            // 4. 大疆 3508 控制
//
//	            	  	        float motor_angle=200.0f;
//
//
//	            	  	        Motors[1].target_angle = (int64_t)(motor_angle * 19.0f * 8192.0f / 360.0f);
//	            	  	      	PID_Calc_Position(1, Motors[1].target_angle);
//        	   send_current();
               Task_System_State_Update();  //姿态角
               HAL_Delay(1);

//
//	  	             //  1. 达妙控制
//	  	        	  //         （ID， Pos，Vel，   Kp，   Kd，Torq:  0.0f (前馈扭矩0)）
//	  	    	//全部设为 0：位置0, 速度0, Kp=0(无刚度), Kd=0(无阻尼), T=0(无前馈力矩)
//	  	    	DM_Send_Ctrl(0x01, 0.0f, 0.0f, 50.0f, 0.5f, 0.0f);
//	  	    	DM_Send_Ctrl(0x02, 0.0f, 0.0f, 50.0f, 0.5f, 0.0f);
//	  	    	DM_Send_Ctrl(0x03, 0.0f, 0.0f, 50.0f, 0.5f, 0.0f);
//	  	    	DM_Send_Ctrl(0x04, 0.0f, 0.0f, 50.0f, 0.5f, 0.0f);

	  	    	// 1. 调用正解函数
	  	    	run_arm_kinematics();

//	            // 2.1 视觉任务状态机 (内部会按需调用 run_arm_to_pos)
//	            Task_Vision_State_Machine();
//	            // 2.2 遥控器控制
//	  	    	arm_knob_direct_control();
//
//	            // 【模式一：零力拖拽 / 重力悬浮模式】 -> 开启此项进行测试
//	            run_arm_drag_teach_mode();

//	  	    	Para_adjust(); // 串口调参函数
//	  	    	Task_Arm_Control();
//	  	    	Task_Arm_Control_Line();
//	  	    	Task_Arm_Control_Line_x();
//	  	        	 //达妙电机组
//	  	        for(uint16_t id = 0x01; id <= 0x04; id++) {
//	  	        	        // 示例：将所有电机控制在 0 弧度位置
//	  	        	        // 参数含义: ID, 位置, 速度, Kp, Kd, 扭矩
//	  	        	      DM_Send_Ctrl(id, 0.0f, 0.0f, 50.0f, 0.5f, 0.0f);
//	  	        	 }



	        }

//
//	  	  // 2. 低频运动学解算调度 (20ms 周期，完全非阻塞)
//	  	  	      	  if(now - last_kinematics_tick >= 20)
//	  	  	      	  {
//	  	  	      	      last_kinematics_tick = now;
//	  	  	      	      AS01_rx(&ctrl1,&ctrl2,&gait,startTime,angles,now);
//
//	  	  	      	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == &huart2) {
//        RS485_RxMode1();
        HAL_UART_Receive_DMA(ctrl1.uart,
                            (uint8_t*)&ctrl1.datas[ctrl1.last_sent_motor_id].motor_recv_data,
                             sizeof(ctrl1.datas[ctrl1.last_sent_motor_id].motor_recv_data));

    }
    if(huart == &huart3) {
//        RS485_RxMode2();
        HAL_UART_Receive_DMA(ctrl2.uart,
                            (uint8_t*)&ctrl2.datas[ctrl2.last_sent_motor_id].motor_recv_data,
                             sizeof(ctrl2.datas[ctrl2.last_sent_motor_id].motor_recv_data));

    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
    	extract_data(&ctrl1.datas[ctrl1.last_sent_motor_id]); // 处理数据
    	ctrl1.dma_busy = 0;
    }
    if (huart == &huart3) {
    	extract_data(&ctrl2.datas[ctrl2.last_sent_motor_id]); // 处理数据
    	ctrl2.dma_busy = 0;
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart== &huart3)
    {
        extern MotorController ctrl2; // 换成你实际的变量名
        ctrl2.dma_busy = 0;           // 强制解除锁定
//        RS485_RxMode2();                   // 恢复接收模式
    }
    if (huart== &huart2)
    {
        extern MotorController ctrl1; // 换成你实际的变量名
        ctrl1.dma_busy = 0;           // 强制解除锁定
//        RS485_RxMode1();                   // 恢复接收模式
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
