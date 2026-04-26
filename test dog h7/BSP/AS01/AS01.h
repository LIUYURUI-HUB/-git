/*
 * AS01.h (接收端)
 *
 * Created on: Mar 7, 2026
 * Author: 22569
 */

#ifndef INC_AS01_H_
#define INC_AS01_H_

#include "stm32h7xx_hal.h"

/* 硬件引脚宏定义 - 必须在 CubeMX 中将 PA8 和 PA11 设为 Output Push-Pull */
#define NRF_CE_GPIO_PORT    GPIOE
#define NRF_CE_PIN          GPIO_PIN_0
#define NRF_CSN_GPIO_PORT   GPIOE
#define NRF_CSN_PIN         GPIO_PIN_1

/* 引脚控制宏 */
#define NRF_CE_LOW()      HAL_GPIO_WritePin(NRF_CE_GPIO_PORT, NRF_CE_PIN, GPIO_PIN_RESET)
#define NRF_CE_HIGH()     HAL_GPIO_WritePin(NRF_CE_GPIO_PORT, NRF_CE_PIN, GPIO_PIN_SET)
#define NRF_CSN_LOW()     HAL_GPIO_WritePin(NRF_CSN_GPIO_PORT, NRF_CSN_PIN, GPIO_PIN_RESET)
#define NRF_CSN_HIGH()    HAL_GPIO_WritePin(NRF_CSN_GPIO_PORT, NRF_CSN_PIN, GPIO_PIN_SET)

/* 遥控器数据包结构体 (必须与发送端完全一致) */
typedef struct __attribute__((packed)) {
    uint16_t joy_lx, joy_ly; // 左摇杆
    uint16_t joy_rx, joy_ry; // 右摇杆
    uint16_t knob[4];        // 4个旋钮
    uint8_t  button[8];      // 4个按键
} RemoteData_t;

/* 指令集与寄存器定义 */
#define CONFIG          0x00  // 配置寄存器
#define EN_AA           0x01  // 自动应答
#define EN_RXADDR       0x02  // 接收通道允许
#define SETUP_RETR      0x04  // 建立重发
#define RF_CH           0x05  // 射频通道
#define RF_SETUP        0x06  // 射频寄存器
#define STATUS          0x07  // 状态寄存器

/* 状态标志位 */
#define MAX_RT          0x10  // 达到最大重发中断
#define TX_DS           0x20  // 发送完成中断
#define RX_OK           0x40  // 接收数据就绪中断标志 (RX_DR)

/* 地址与负载指令 */
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_ADDR_P0      0x0A  // 通道0接收地址寄存器
#define RX_PW_P0        0x11  // 通道0接收数据长度 (必须设为32)

#define RD_RX_PLOAD     0x61  // 读取RX有效载荷指令
#define WR_TX_PLOAD     0xA0  // 写TX有效载荷
#define FLUSH_TX        0xE1  // 清除TX FIFO
#define FLUSH_RX        0xE2  // 清除RX FIFO

/* 函数声明 */
void NRF24L01_Init(void);                      // 接收初始化
uint8_t NRF24L01_Check(void);                  // 模块连接检测
uint8_t NRF24L01_Check_SPI(void);              // SPI 物理层检测
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);     // 接收数据包函数

#endif /* INC_AS01_H_ */
