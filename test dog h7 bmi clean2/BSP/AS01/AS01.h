/*
 * AS01.h (接收�?
 *
 * Created on: Mar 7, 2026
 * Author: 22569
 */

#ifndef INC_AS01_H_
#define INC_AS01_H_

#include "stm32h7xx_hal.h"

/* 硬件引脚宏定�?- 必须�?CubeMX 中将 PA8 �?PA11 设为 Output Push-Pull */
#define NRF_CE_GPIO_PORT    GPIOE
#define NRF_CE_PIN          GPIO_PIN_0
#define NRF_CSN_GPIO_PORT   GPIOE
#define NRF_CSN_PIN         GPIO_PIN_1

/* 引脚控制�?*/
#define NRF_CE_LOW()      HAL_GPIO_WritePin(NRF_CE_GPIO_PORT, NRF_CE_PIN, GPIO_PIN_RESET)
#define NRF_CE_HIGH()     HAL_GPIO_WritePin(NRF_CE_GPIO_PORT, NRF_CE_PIN, GPIO_PIN_SET)
#define NRF_CSN_LOW()     HAL_GPIO_WritePin(NRF_CSN_GPIO_PORT, NRF_CSN_PIN, GPIO_PIN_RESET)
#define NRF_CSN_HIGH()    HAL_GPIO_WritePin(NRF_CSN_GPIO_PORT, NRF_CSN_PIN, GPIO_PIN_SET)

/* Remote packet layout: must match transmitter exactly. */
typedef struct __attribute__((packed)) {
    uint16_t joy_lx, joy_ly;
    uint16_t joy_rx, joy_ry;
    uint16_t knob[4];
    uint8_t  buttons;
} RemoteData_t;

/* 指令集与寄存器定�?*/
#define CONFIG          0x00  // 配置寄存�?
#define EN_AA           0x01  // 自动应答
#define EN_RXADDR       0x02  // 接收通道允许
#define SETUP_RETR      0x04  // 建立重发
#define RF_CH           0x05  // 射频通道
#define RF_SETUP        0x06  // 射频寄存�?
#define STATUS          0x07  // 状态寄存器

/* 状态标志位 */
#define MAX_RT          0x10  // 达到最大重发中�?
#define TX_DS           0x20  // 发送完成中�?
#define RX_OK           0x40  // 接收数据就绪中断标志 (RX_DR)

/* 地址与负载指�?*/
#define TX_ADDR         0x10  // 发送地址寄存�?
#define RX_ADDR_P0      0x0A  // 通道0接收地址寄存�?
#define RX_PW_P0        0x11  // 通道0接收数据长度 (必须设为32)

#define RD_RX_PLOAD     0x61  // 读取RX有效载荷指令
#define WR_TX_PLOAD     0xA0  // 写TX有效载荷
#define FLUSH_TX        0xE1  // 清除TX FIFO
#define FLUSH_RX        0xE2  // 清除RX FIFO

/* 函数声明 */
void NRF24L01_Init(void);                      // 接收初始�?
uint8_t NRF24L01_Check(void);                  // 模块连接检�?
uint8_t NRF24L01_Check_SPI(void);              // SPI 物理层检�?
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);     // 接收数据包函�?

extern volatile uint8_t g_NRF_CheckSpiResult;
extern volatile uint8_t g_NRF_InitCalled;
extern volatile uint8_t g_NRF_LastRxStatus;
extern volatile uint8_t g_NRF_LastRxOk;

#endif /* INC_AS01_H_ */
