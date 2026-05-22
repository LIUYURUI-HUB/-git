#ifndef __BMI088_DRIVER_H
#define __BMI088_DRIVER_H

#include "main.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"

/* ========== 全局变量声明 ========== */
extern float BMI088_ACCEL_SEN;
extern float BMI088_GYRO_SEN;

/* ========== SPI 读写宏定义 ========== */
#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }

#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }


#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_muli_reg((reg), (data), (len));\
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }

#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

#elif defined(BMI088_USE_IIC)
/* IIC 宏定义占位 */
#endif

/* ========== 函数声明 ========== */
uint8_t BMI088_init(void);
uint8_t bmi088_accel_init(void);
uint8_t bmi088_gyro_init(void);
void BMI088_read(float gyro[3], float accel[3], float *temperate);

/* 底层静态函数声明 */
void BMI088_write_single_reg(uint8_t reg, uint8_t data);
void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#endif
