#ifndef SIM_HOST_DM_DRIVER_H
#define SIM_HOST_DM_DRIVER_H

#include "main.h"
#include <stdint.h>

typedef struct {
    uint8_t ID;
    uint8_t ERR;
    float POS;
    float VEL;
    float T;
    int8_t T_MOS;
    int8_t T_Rotor;
    float Filter_VEL;
} DM_Motor_t;

extern DM_Motor_t Arm_Motors[4];

uint8_t DM_Send_Ctrl(uint16_t id, float p_des, float v_des, float Kp, float Kd, float t_ff);

#endif
