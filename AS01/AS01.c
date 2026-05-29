/*
 * AS01_RX.c (终极优化版)
 * 针对 STM32H7 的 GPIO 高翻转率、SPI 溢出特性进行了深度保护
 */

#include "AS01.h"

extern SPI_HandleTypeDef hspi1;

// 接收地址
const uint8_t RX_ADDRESS[5] = {0x34, 0x43, 0x10, 0x10, 0x01}; // 任务赛

// ==========================================================
// 调试专用变量：在 STM32CubeIDE 的 Live Expressions 中添加此变量
// 用来观察读回的数据到底是什么，以判断底层硬件故障原因
uint8_t g_NRF_Read_Debug[5] = {0};
volatile uint8_t g_NRF_CheckSpiResult = 0xFF;
volatile uint8_t g_NRF_InitCalled = 0;
volatile uint8_t g_NRF_LastRxStatus = 0;
volatile uint8_t g_NRF_LastRxOk = 0;
volatile uint8_t g_NRF_RegConfig = 0;
volatile uint8_t g_NRF_RegEnAa = 0;
volatile uint8_t g_NRF_RegEnRxAddr = 0;
volatile uint8_t g_NRF_RegRfCh = 0;
volatile uint8_t g_NRF_RegRfSetup = 0;
volatile uint8_t g_NRF_RegRxPwP0 = 0;
volatile uint8_t g_NRF_RegFifoStatus = 0;
volatile uint8_t g_NRF_CePinState = 0;
volatile uint8_t g_NRF_RxAddrP0_Debug[5] = {0};
// ==========================================================

/**
 * @brief H7 极速 GPIO 纳秒级延迟补偿
 * NRF24L01 要求 CSN 下降沿后至少等待 5ns 再发数据，CSN 拉高需要保持至少 50ns。
 * H7 主频高达 400MHz+，连续拉低拉高会违背时序，必须人为插入 __NOP() 消耗时间。
 */
static void H7_SPI_Delay(void) {
    for(volatile int i = 0; i < 50; i++) {
        __NOP();
    }
}

// 安全的 CSN 拉低
static void NRF_CSN_LOW_SAFE(void) {
    NRF_CSN_LOW();
    H7_SPI_Delay();
}

// 安全的 CSN 拉高
static void NRF_CSN_HIGH_SAFE(void) {
    H7_SPI_Delay();
    NRF_CSN_HIGH();
    H7_SPI_Delay();
}

/**
 * @brief SPI 读写单字节 (带 H7 OVR 错误清除防卡死)
 */
static uint8_t SPI2_ReadWriteByte(uint8_t byte) {
    uint8_t d_read = 0xFF;

    // H7 特有坑：如果 SPI 发生 Overrun (溢出) 错误，会导致后续读写永远返回 0xFF 或卡死
    // 每次通信前检查并清除标志位
    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_OVR)) {
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    }

    // 使用阻塞式传输，设置合理的 10ms 超时
    if(HAL_SPI_TransmitReceive(&hspi1, &byte, &d_read, 1, 10) != HAL_OK) {
        return 0xFF;
    }
    return d_read;
}

// 写寄存器
void NRF_Write_Reg(uint8_t reg, uint8_t value) {
    NRF_CSN_LOW_SAFE();
    SPI2_ReadWriteByte(reg | 0x20); // 0x20 为写指令前缀
    SPI2_ReadWriteByte(value);
    NRF_CSN_HIGH_SAFE();
}

// 读寄存器
uint8_t NRF_Read_Reg(uint8_t reg) {
    uint8_t value;
    NRF_CSN_LOW_SAFE();
    SPI2_ReadWriteByte(reg);
    value = SPI2_ReadWriteByte(0xFF); // 发送空字节读取数据
    NRF_CSN_HIGH_SAFE();
    return value;
}

// 写缓冲区
void NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len) {
    NRF_CSN_LOW_SAFE();
    SPI2_ReadWriteByte(reg);
    for(uint8_t i=0; i<len; i++) {
        SPI2_ReadWriteByte(pBuf[i]);
    }
    NRF_CSN_HIGH_SAFE();
}

// 读缓冲区
void NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len) {
    NRF_CSN_LOW_SAFE();
    SPI2_ReadWriteByte(reg);
    for(uint8_t i=0; i<len; i++) {
        pBuf[i] = SPI2_ReadWriteByte(0xFF);
    }
    NRF_CSN_HIGH_SAFE();
}

static void NRF24L01_UpdateDebugRegs(void) {
    g_NRF_RegConfig = NRF_Read_Reg(CONFIG);
    g_NRF_RegEnAa = NRF_Read_Reg(EN_AA);
    g_NRF_RegEnRxAddr = NRF_Read_Reg(EN_RXADDR);
    g_NRF_RegRfCh = NRF_Read_Reg(RF_CH);
    g_NRF_RegRfSetup = NRF_Read_Reg(RF_SETUP);
    g_NRF_RegRxPwP0 = NRF_Read_Reg(RX_PW_P0);
    g_NRF_RegFifoStatus = NRF_Read_Reg(FIFO_STATUS);
    g_NRF_CePinState = (HAL_GPIO_ReadPin(NRF_CE_GPIO_PORT, NRF_CE_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    NRF_Read_Buf(RX_ADDR_P0, (uint8_t*)g_NRF_RxAddrP0_Debug, 5);
}

/**
 * @brief 物理层自检改进版
 * NRF24L01 的 TX_ADDR 寄存器是可以读写的，用它来测试 SPI 是否通畅最有效
 */
uint8_t NRF24L01_Check_SPI(void) {
    uint8_t test_addr[5] = {0xA5, 0xA1, 0xA2, 0xA3, 0xA4};
    uint8_t read_addr[5] = {0};

    NRF_CE_LOW(); // 自检时务必拉低 CE，进入待机模式
    H7_SPI_Delay();

    // 1. 尝试写入测试地址到 TX_ADDR (0x10)
    NRF_Write_Buf(0x20 | 0x10, test_addr, 5);

    // 2. 读出来对比
    NRF_Read_Buf(0x10, read_addr, 5);

    // ==========================================
    // 诊断代码：把读到的数据保存到全局变量以便观察
    for(int i = 0; i < 5; i++) {
        g_NRF_Read_Debug[i] = read_addr[i];
    }
    // ==========================================

    // 3. 逐位对比
    for(int i=0; i<5; i++) {
        if(read_addr[i] != test_addr[i]) {
            g_NRF_CheckSpiResult = 1;
            return 1; // 失败
        }
    }
    g_NRF_CheckSpiResult = 0;
    NRF24L01_UpdateDebugRegs();
    return 0; // 成功
}

/**
 * @brief 替代用户添加的 Check 函数，逻辑与 Check_SPI 相同
 */
uint8_t NRF24L01_Check(void) {
    uint8_t buf[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t read_buf[5] = {0};

    NRF_CE_LOW(); // 必须拉低 CE
    H7_SPI_Delay();

    // 0x10 是 TX_ADDR 的寄存器地址
    NRF_Write_Buf(0x20 | 0x10, buf, 5);
    NRF_Read_Buf(0x10, read_buf, 5);

    // ==========================================
    // 诊断代码：把读到的数据保存到全局变量以便观察
    for(int i = 0; i < 5; i++) {
        g_NRF_Read_Debug[i] = read_buf[i];
    }
    // ==========================================

    for (int i = 0; i < 5; i++) {
        if (buf[i] != read_buf[i]) {
            return 1; // 不在线
        }
    }
    return 0; // 在线
}

void NRF24L01_Init(void) {
    NRF_CE_LOW();
    g_NRF_InitCalled = 1;

    // 基础配置
    NRF_Write_Buf(0x20|RX_ADDR_P0, (uint8_t*)RX_ADDRESS, 5);
    NRF_Write_Reg(EN_AA, 0x01);
    NRF_Write_Reg(EN_RXADDR, 0x01);
    NRF_Write_Reg(SETUP_RETR, 0x1A);
    NRF_Write_Reg(RF_CH, 40);
    NRF_Write_Reg(RX_PW_P0, sizeof(RemoteData_t));

    // 重点：RF_SETUP 速率
    // 0x26 表示 250kbps, 0dBm (信号最强)
    // 如果电源不稳定，建议使用 0x20 (250kbps, -18dBm)
    NRF_Write_Reg(RF_SETUP, 0x26);

    NRF_Write_Reg(CONFIG, 0x0F);      // 开启 CRC, 上电, 接收模式

    // 复位状态寄存器（清除旧中断）
    NRF_Write_Reg(STATUS, 0x70);

    // 清空接收缓冲区，防止旧数据干扰
    NRF_CSN_LOW_SAFE();
    SPI2_ReadWriteByte(0xE2); // FLUSH_RX 指令
    NRF_CSN_HIGH_SAFE();

    NRF_CE_HIGH();
    HAL_Delay(10); // 等待稳定
    NRF24L01_UpdateDebugRegs();
}

/**
 * @brief 接收数据包
 * @param rxbuf 接收缓冲区，长度至少32字节
 * @return 0: 接收成功, 1: 无数据或接收失败
 */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf) {
    uint8_t status = NRF_Read_Reg(STATUS);
    g_NRF_LastRxStatus = status;
    g_NRF_LastRxOk = (status & RX_OK) ? 1U : 0U;
    g_NRF_RegFifoStatus = NRF_Read_Reg(FIFO_STATUS);
    g_NRF_CePinState = (HAL_GPIO_ReadPin(NRF_CE_GPIO_PORT, NRF_CE_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    
    // 检查是否有接收数据就绪标志 (RX_OK 通常是 0x40)
    if (status & RX_OK) {
        // 读取接收到的数据 (0x61 = RD_RX_PLOAD)
        NRF_Read_Buf(0x61, rxbuf, sizeof(RemoteData_t));
        
        // 清除接收数据就绪标志
        NRF_Write_Reg(STATUS, status | RX_OK);
        
        // 清空接收缓冲区
        NRF_CSN_LOW_SAFE();
        SPI2_ReadWriteByte(0xE2); // FLUSH_RX 指令
        NRF_CSN_HIGH_SAFE();
        
        return 0; // 接收成功
    }
    
    return 1; // 无数据
}
