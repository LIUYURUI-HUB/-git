#include "motor_controller.h"
#include <string.h>
#include "rs485.h"

 MotorController ctrl[4] __attribute__((section(".dma_buffer")));
// MotorController ctrl[4];

/**
  * @brief 初始化电机控制器
  */
void MotorController_Init(MotorController* ctrl,
                         UART_HandleTypeDef* uart,
                         GPIO_TypeDef* de_port,
                         uint16_t de_pin) {
    // 初始化硬件参数
    ctrl->uart = uart;
    ctrl->de_port = de_port;
    ctrl->de_pin = de_pin;
    ctrl->dma_busy = 0;
    ctrl->last_sent_motor_id = 0;
    ctrl->last_send_tick = 0;


    // 初始化所有电机指令
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        ctrl->cmds[i].id = i;     // ID从0开始
        ctrl->cmds[i].mode = 1;       // 默认阻尼模式
        ctrl->cmds[i].T = 0.0f;       // 扭矩
        ctrl->cmds[i].W = 0.0f;       // 速度
        ctrl->cmds[i].Pos = 0.0f;     // 位置
        ctrl->cmds[i].K_P = 0.0f;     // 刚度
        ctrl->cmds[i].K_W = 0.0f;     // 阻尼

        modify_data(&ctrl->cmds[i]); // 生成协议数据

        memset(&ctrl->datas[i], 0, sizeof(MotorData_t));
        ctrl->datas[i].motor_id = i;

    }


}



/**
  * @brief 设置电机控制参数
  */
void MotorController_SetCommand(MotorController* ctrl,
                              uint8_t motor_id,
                              uint8_t mode,
                              float torque,
                              float speed,
                              float position,
                              float k_p,
                              float k_w) {
    if ( motor_id > MOTOR_NUM) return;

    uint8_t idx = motor_id;
    ctrl->cmds[idx].mode = mode;
    ctrl->cmds[idx].T = torque;
    ctrl->cmds[idx].W = speed;
    ctrl->cmds[idx].Pos = position;
    ctrl->cmds[idx].K_P = k_p;
    ctrl->cmds[idx].K_W = k_w;

    modify_data(&ctrl->cmds[idx]);     // 更新协议数据
}

/**
  * @brief 发送单个电机指令
  * @return 0成功, -1失败
  */
int MotorController_SendCommand1(MotorController* ctrl, uint8_t motor_id) {
    if (ctrl->dma_busy) {
        if (HAL_GetTick() - ctrl->last_send_tick > 5) {
            ctrl->datas[ctrl->last_sent_motor_id].correct = 0;
            ctrl->dma_busy = 0;
            HAL_UART_Abort(ctrl->uart);
        } else {
            return -1;
        }
    }
	if (motor_id >= MOTOR_NUM || ctrl->dma_busy) {
        return -1; // 电机ID无效或DMA正忙
    }

    ctrl->last_sent_motor_id = motor_id;
    // 设置发送模式 (拉高DE引脚)
//    RS485_TxMode1();

    // 标记DMA忙碌状态
    ctrl->dma_busy = 1;
    ctrl->last_send_tick = HAL_GetTick();

    // 启动DMA发送
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        ctrl->uart,
        (uint8_t*)&ctrl->cmds[motor_id].motor_send_data,
        sizeof(ctrl->cmds[motor_id].motor_send_data));
    if (status != HAL_OK) {
        ctrl->dma_busy = 0; // 回滚状态，防止死锁

        // 可选：将RS485切回接收模式，防止一直占用总线
        // RS485_RxMode1();

        return -1;
    }

    return 0; // 发送启动成功
}
//int MotorController_SendCommand1(MotorController* ctrl, uint8_t motor_id) {
//    if (motor_id >= MOTOR_NUM || ctrl->dma_busy) {
//        return -1; // 电机ID无效或DMA正忙
//    }
//
//    ctrl->last_sent_motor_id = motor_id;
//    // 设置发送模式
//    RS485_TxMode1();
//
//    // 标记DMA忙碌状态
//    ctrl->dma_busy = 1;
//
//    // 启动DMA发送
//    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
//        ctrl->uart,
//        (uint8_t*)&ctrl->cmds[motor_id].motor_send_data,
//        sizeof(ctrl->cmds[motor_id].motor_send_data));
//
//    return (status == HAL_OK) ? 0 : -1;
//}

/**
  * @brief 发送单个电机指令
  * @return 0成功, -1失败
  */
int MotorController_SendCommand2(MotorController* ctrl, uint8_t motor_id) {

    if (ctrl->dma_busy) {
        if (HAL_GetTick() - ctrl->last_send_tick > 5) {
            ctrl->datas[ctrl->last_sent_motor_id].correct = 0;
            ctrl->dma_busy = 0;
            HAL_UART_Abort(ctrl->uart);
        } else {
            return -1;
        }
    }
	if (motor_id >= MOTOR_NUM || ctrl->dma_busy) {
        return -1; // 电机ID无效或DMA正忙
    }

    ctrl->last_sent_motor_id = motor_id;
    // 设置发送模式 (拉高DE引脚)
//    RS485_TxMode1();

    // 标记DMA忙碌状态
    ctrl->dma_busy = 1;
    ctrl->last_send_tick = HAL_GetTick();

    // 启动DMA发送
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        ctrl->uart,
        (uint8_t*)&ctrl->cmds[motor_id].motor_send_data,
        sizeof(ctrl->cmds[motor_id].motor_send_data));
    if (status != HAL_OK) {
        ctrl->dma_busy = 0; // 回滚状态，防止死锁

        // 可选：将RS485切回接收模式，防止一直占用总线
        // RS485_RxMode1();

        return -1;
    }

    return 0; // 发送启动成功
}
//int MotorController_SendCommand2(MotorController* ctrl, uint8_t motor_id) {
//    // 参数检查
//    if (motor_id >= MOTOR_NUM) {
//        return -1; // 无效电机ID
//    }
//
////    uint32_t timeout = HAL_GetTick() + 2;
//
//	ctrl->last_sent_motor_id = motor_id;
//    // 设置发送模式
//    RS485_TxMode2();
//
//    // 标记DMA忙碌状态
//    ctrl->dma_busy = 1;
//
//    // 启动DMA发送
//    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
//        ctrl->uart,
//        (uint8_t*)&ctrl->cmds[motor_id].motor_send_data,
//        sizeof(ctrl->cmds[motor_id].motor_send_data));
//
//    return (status == HAL_OK) ? 0 : -1;
//}

