#include "Para_adjust.h"
#include "dm_driver.h"
#include "arm_g.h"
#include "arm.h"

// ==============================================================================
// 声明在 arm.c 中定义的外部变量
// ==============================================================================
extern DM_Motor_t Arm_Motors[4];

extern UART_HandleTypeDef huart7;
extern ArmDebug_t arm_debug;
// VOFA+ 发送数据帧结构体缓存区
// (注意: 需要在你的 arm_task.h 的 ArmLogFrame 结构体中添加这8个新增的浮点变量)
ArmLogFrame arm_frame __attribute__((section(".dma_buffer")));

//VOFA+ 状态上报发送任务 (100Hz)

void Para_adjust(void) {
    static uint32_t last_send_tick = 0;
    static uint8_t is_init = 0;

    // 1. 初始化帧头帧尾 (仅执行一次)
    if (!is_init) {
        arm_frame.header[0] = 0xAA;
        arm_frame.header[1] = 0x55;
        arm_frame.tail[0] = 0x55;
        arm_frame.tail[1] = 0xAA;
        is_init = 1;
    }

    uint32_t now = HAL_GetTick();

    // 2. 限制发送频率为 100Hz (每 10ms 触发一次发送)
    // 刚开机的前 3000ms 不发送数据，等待系统稳定
    if (now > 3000 && (now - last_send_tick >= 10)) {
        last_send_tick = now;

        if(huart7.gState == HAL_UART_STATE_READY) {
            arm_frame.time_us = now * 1000; // 转换为微秒

            // 记录物理真实反馈
            // 填入 8 个关键 float 数据
            arm_frame.m2_pos = Arm_Motors[1].POS;
            arm_frame.m3_pos = Arm_Motors[2].POS;
            arm_frame.m2_vel = Arm_Motors[1].VEL;
            arm_frame.m3_vel = Arm_Motors[2].VEL;

            // 注意: 达妙电机的力矩变量通常为 T。如果编译依然提示找不到，请去 dm_driver.h 里看下是不是叫 torq 或者 torque。
            arm_frame.m2_tau_actual = Arm_Motors[1].T;
            arm_frame.m3_tau_actual = Arm_Motors[2].T;

            // 用 0.0f 凑够 8 个 float 占位，替代废弃的 tau_ff
            arm_frame.reserved1 = arm_debug.tau_ff_m2;
            arm_frame.reserved2 = arm_debug.tau_ff_m3;

            // DMA发送
            HAL_UART_Transmit_DMA(&huart7, (uint8_t*)&arm_frame, sizeof(ArmLogFrame));
        }
    }
}
