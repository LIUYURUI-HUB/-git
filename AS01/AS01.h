/*
 * AS01.h
 *
 * NRF24L01/AS01 receiver interface.
 */

#ifndef INC_AS01_H_
#define INC_AS01_H_

#include "stm32h7xx_hal.h"

#define NRF_CE_GPIO_PORT    GPIOE
#define NRF_CE_PIN          GPIO_PIN_15
#define NRF_CSN_GPIO_PORT   GPIOE
#define NRF_CSN_PIN         GPIO_PIN_14

#define NRF_CE_LOW()        HAL_GPIO_WritePin(NRF_CE_GPIO_PORT, NRF_CE_PIN, GPIO_PIN_RESET)
#define NRF_CE_HIGH()       HAL_GPIO_WritePin(NRF_CE_GPIO_PORT, NRF_CE_PIN, GPIO_PIN_SET)
#define NRF_CSN_LOW()       HAL_GPIO_WritePin(NRF_CSN_GPIO_PORT, NRF_CSN_PIN, GPIO_PIN_RESET)
#define NRF_CSN_HIGH()      HAL_GPIO_WritePin(NRF_CSN_GPIO_PORT, NRF_CSN_PIN, GPIO_PIN_SET)

typedef struct __attribute__((packed)) {
    uint16_t joy_lx, joy_ly;
    uint16_t joy_rx, joy_ry;
    uint16_t knob[4];
    uint8_t  buttons;
} RemoteData_t;

#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07

#define MAX_RT          0x10
#define TX_DS           0x20
#define RX_OK           0x40

#define RX_ADDR_P0      0x0A
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define FIFO_STATUS     0x17

#define RD_RX_PLOAD     0x61
#define WR_TX_PLOAD     0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2

void NRF24L01_Init(void);
uint8_t NRF24L01_Check(void);
uint8_t NRF24L01_Check_SPI(void);
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);

extern uint8_t g_NRF_Read_Debug[5];
extern volatile uint8_t g_NRF_CheckSpiResult;      /* 0=pass, 1=fail, 0xFF=not checked */
extern volatile uint8_t g_NRF_InitCalled;          /* 1=NRF24L01_Init() has run */
extern volatile uint8_t g_NRF_LastRxStatus;        /* last STATUS register */
extern volatile uint8_t g_NRF_LastRxOk;            /* 1=last poll saw RX_DR */
extern volatile uint8_t g_NRF_RegConfig;           /* expected 0x0F */
extern volatile uint8_t g_NRF_RegEnAa;             /* expected 0x01 */
extern volatile uint8_t g_NRF_RegEnRxAddr;         /* expected 0x01 */
extern volatile uint8_t g_NRF_RegRfCh;             /* expected 40 */
extern volatile uint8_t g_NRF_RegRfSetup;          /* expected 0x26 */
extern volatile uint8_t g_NRF_RegRxPwP0;           /* expected sizeof(RemoteData_t) */
extern volatile uint8_t g_NRF_RegFifoStatus;       /* bit0=RX_EMPTY, bit1=RX_FULL */
extern volatile uint8_t g_NRF_CePinState;          /* 1=CE high, 0=CE low */
extern volatile uint8_t g_NRF_RxAddrP0_Debug[5];   /* expected 34 43 10 10 01 */

#endif /* INC_AS01_H_ */
