#ifndef __DM_DRIVER_H__
#define __DM_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 达妙电机反馈数据结构体
 * @note  根据 DM_RX_Decode 函数解析逻辑定义
 */
typedef struct {
    uint8_t  ID;        // 电机ID (4 bits)
    uint8_t  ERR;       // 错误码 (4 bits)
    float    POS;       // 当前位置 (rad)
    float    VEL;       // 当前速度 (rad/s)
    float    T;         // 当前扭矩 (N.m)
    int8_t   T_MOS;     // 驱动板MOS平均温度 (℃)
    int8_t   T_Rotor;   // 电机线圈平均温度 (℃)
    float    Filter_VEL;  //滤波后的速度
} DM_Motor_t;

/* Exported variables --------------------------------------------------------*/
extern DM_Motor_t dm_back;          // 声明外部定义的电机反馈结构体(单电机)
extern DM_Motor_t Arm_motors[4];   //机械臂电机
extern FDCAN_HandleTypeDef hfdcan2; // 声明外部定义的FDCAN句柄

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化CAN通信及滤波器
 */
void FDCAN2_Filter_Init(void);

/**
 * @brief  使能电机
 * @param  id: 电机CAN ID
 */
void DM_Motor_Enable(uint16_t id);

/**
 * @brief  失能电机
 * @param  id: 电机CAN ID
 */
void DM_Motor_Disable(uint16_t id);

// 设置当前物理位置为电机零点 (每次上电执行)
void DM_Set_Zero_Pos(uint16_t id);
/**
 * @brief  发送MIT模式控制指令
 * @param  id:    电机CAN ID
 * @param  p_des: 期望位置 (rad)
 * @param  v_des: 期望速度 (rad/s)
 * @param  Kp:    位置增益
 * @param  Kd:    速度增益
 * @param  t_ff:  前馈扭矩 (N.m)
 * @retval 0: 发送成功, 1: 发送失败
 */
uint8_t DM_Send_Ctrl(uint16_t id, float p_des, float v_des, float Kp, float Kd, float t_ff);

/**
 * @brief  解码接收到的CAN数据
 * @param  data: 接收到的8字节原始数据数组
 */
void DM_RX_Decode(uint8_t* data, uint16_t can_id);

#ifdef __cplusplus
}
#endif

#endif /* __DM_DRIVER_H__ */
