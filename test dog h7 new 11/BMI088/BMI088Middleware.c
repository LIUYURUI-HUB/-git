#include "BMI088Middleware.h"
#include "main.h"

/* ========== SPI 句柄定义 ========== */
#define BMI088_USING_SPI_UNIT   hspi2
extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

/* ========== GPIO 初始化 ========== */
void BMI088_GPIO_init(void)
{
    /*
     * 注意：GPIO 初始化实际上在 CubeMX 生成的 MX_GPIO_Init() 中完成
     * 确保 ACC_CS_Pin 和 GYRO_CS_Pin 在 main.h 中已定义
     */
}

/* ========== 通信初始化 ========== */
void BMI088_com_init(void)
{
    /*
     * 注意：SPI 初始化在 CubeMX 生成的 MX_SPI2_Init() 中完成
     * 确保 SPI2 配置为 Mode 3 (CPOL=High, CPHA=2Edge)
     */
}

/* ========== 毫秒延时 ========== */
void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}

/* ========== 微秒延时（兼容 H7 主频） ========== */
void BMI088_delay_us(uint16_t us)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;

    reload = SysTick->LOAD;
    ticks = us * (SystemCoreClock / 1000000);
    told = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

/* ========== 片选控制 ========== */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

/* ========== SPI 单字节读写 ========== */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
