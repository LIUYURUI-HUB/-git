#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "AS01.h"
#include "kinematics.h"
#include "gait.h"
#include "motor_controller.h"
#include "3508_driver.h"
#include "string.h"
#include "dm_driver.h"

#define COMPETITION_AUTO_MODE 0
#define MODE_TOGGLE_BUTTON_INDEX 3U

typedef enum {
    ROBOT_STATE_IDLE = 0,
    ROBOT_STATE_FORWARD,
    ROBOT_STATE_BACKWARD,
    ROBOT_STATE_LEFT,
    ROBOT_STATE_RIGHT
} RobotState_e;

typedef enum {
    GAIT_MODE_TROT,
    GAIT_MODE_WALK,
    GAIT_MODE_BOUND,
    GAIT_MODE_PRONK
} GaitMode_e;

typedef enum {
    SYS_MODE_RC = 0,
    SYS_MODE_VISION
} SystemCtrlMode_e;

typedef struct {
    float vx;
    float vy;
    float wz;
    RobotState_e state;
} ChassisCmd_t;

typedef struct {
    uint32_t nrf_rx_count;                    // NRF/AS01 valid packet count.
    uint32_t nrf_timeout_count;               // NRF/AS01 timeout edge count.
    uint32_t nrf_last_rx_time;                // Last NRF valid packet time, HAL_GetTick() ms.

    uint32_t vision_rx_count;                 // Valid USB vision chassis packet count.
    uint32_t vision_timeout_count;            // Vision chassis timeout edge count.
    uint32_t vision_checksum_error_count;     // Vision protocol checksum error count.
    uint32_t vision_last_rx_time;             // Last valid vision chassis packet time, ms.

    uint32_t usart2_rx_count;                 // USART2/ctrl1 Unitree feedback complete count.
    uint32_t usart2_err_count;                // USART2 HAL error callback count.
    uint32_t usart2_last_error;               // Last USART2 HAL error code.
    uint32_t usart3_rx_count;                 // USART3/ctrl2 Unitree feedback complete count.
    uint32_t usart3_err_count;                // USART3 HAL error callback count.
    uint32_t usart3_last_error;               // Last USART3 HAL error code.

    uint32_t fdcan1_rx_count;                 // FDCAN1 receive count.
    uint32_t fdcan1_err_count;                // FDCAN1 receive error count.
    uint32_t fdcan1_last_error;               // Last FDCAN1 HAL error code.
    uint32_t fdcan2_rx_count;                 // FDCAN2 receive count.
    uint32_t fdcan2_err_count;                // FDCAN2 receive error count.
    uint32_t fdcan2_last_error;               // Last FDCAN2 HAL error code.
    uint32_t fdcan3_rx_count;                 // FDCAN3 receive count.
    uint32_t fdcan3_err_count;                // FDCAN3 receive error count.
    uint32_t fdcan3_last_error;               // Last FDCAN3 HAL error code.

    uint32_t loop_5ms_count;                  // 5 ms control-loop execution count.
    uint32_t loop_overrun_count;              // Count of control loop intervals over limit.
    uint32_t max_loop_dt_ms;                  // Max observed control-loop interval, ms.
} RobotDiag_t;

extern volatile RobotDiag_t g_robot_diag;

void state_zero0(void);
void arm_knob_direct_control(void);
void Control_Init(QuadrupedGait* gait);
SystemCtrlMode_e Control_GetSystemMode(void);
void joystick_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now);
void button_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t currentTime);
void AS01_rx(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now);
void state_zero(uint32_t now, float current_kp);

#endif
