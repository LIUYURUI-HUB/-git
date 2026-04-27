// rs485.h
#ifndef RS485_H
#define RS485_H

#include "stm32h7xx_hal.h"  // 确保 HAL_GPIO_WritePin 可用
#include "main.h"
// 定义 RS485 收发控制宏
#define RS485_RxMode1()    HAL_GPIO_WritePin(RS485_REDE1_GPIO_Port, RS485_REDE1_Pin, GPIO_PIN_RESET)
#define RS485_TxMode1()    HAL_GPIO_WritePin(RS485_REDE1_GPIO_Port, RS485_REDE1_Pin, GPIO_PIN_SET)

#define RS485_RxMode2()    HAL_GPIO_WritePin(RS485_REDE2_GPIO_Port, RS485_REDE2_Pin, GPIO_PIN_RESET)
#define RS485_TxMode2()    HAL_GPIO_WritePin(RS485_REDE2_GPIO_Port, RS485_REDE2_Pin, GPIO_PIN_SET)
#endif // RS485_H
