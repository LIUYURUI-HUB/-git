/*
 * Para_adjust.h
 *
 *  Created on: Mar 24, 2026
 *      Author: developer
 */

#ifndef PARA_ADJUST_H_
#define PARA_ADJUST_H_


#include "stdint.h"

// 为了兼容 VOFA+ 的 JustFloat 协议或 Python Struct 解包
// 我们对齐打包该结构体
#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];      // 0xAA 0x55
    uint32_t time_us;       // 时间戳
    float m2_pos;           // M2实际位置
    float m3_pos;           // M3实际位置
    float m2_vel;           // M2实际速度
    float m3_vel;           // M3实际速度
    float m2_tau_actual;    // M2实际力矩
    float m3_tau_actual;    // M3实际力矩
    float reserved1;        // 填0凑数
    float reserved2;        // 填0凑数
    uint8_t tail[2];        // 0x55 0xAA
} ArmLogFrame;
#pragma pack(pop)

void Para_adjust(void);

#endif /* PARA_ADJUST_H_ */
