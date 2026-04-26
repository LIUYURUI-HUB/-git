#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <gom_protocol.h>  // 电机协议定义
#include <stdint.h>
#include "stm32h7xx_hal.h"
// 配置参数
#define MOTOR_NUM         9      // 电机数量
#define CMD_TIMEOUT_MS    10      // 指令发送超时时间

// 电机控制结构体
typedef struct {
	MotorData_t datas[MOTOR_NUM];//每个电机的返回值
    MotorCmd_t cmds[MOTOR_NUM];// 每个电机的指令
    UART_HandleTypeDef* uart;     // 使用的UART句柄
    GPIO_TypeDef* de_port;        // RS485方向控制端口
    uint16_t de_pin;
    volatile uint8_t dma_busy;// RS485方向控制引脚
    uint8_t last_sent_motor_id;
    uint8_t last_recv_motor_id;
    uint32_t last_send_tick;
} MotorController;

// 初始化电机控制器
void MotorController_Init(MotorController* ctrl,
                         UART_HandleTypeDef* uart,
                         GPIO_TypeDef* de_port,
                         uint16_t de_pin
						 );

// 设置单个电机参数
void MotorController_SetCommand(MotorController* ctrl,
                              uint8_t motor_id,
                              uint8_t mode,
                              float torque,
                              float speed,
                              float position,
                              float k_p,
                              float k_w);


// 发送单个电机指令
int MotorController_SendCommand1(MotorController* ctrl,
                              uint8_t motor_id);

int MotorController_SendCommand2(MotorController* ctrl,
                              uint8_t motor_id);

// 发送所有电机指令(轮询方式)
void MotorController_SendAllCommands(MotorController* ctrl);
void MotorController_ReceiveData1(MotorController* ctrl);
void MotorController_ReceiveData2(MotorController* ctrl);
#endif // MOTOR_CONTROLLER_H
