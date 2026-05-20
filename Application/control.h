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
    uint32_t nrf_rx_count;              // NRF/AS01有效收包次数；AS01_rx()成功读到遥控包后累加，正常连接时应持续增长。
    uint32_t nrf_timeout_count;         // NRF/AS01失联次数；遥控超过REMOTE_TIMEOUT_MS未更新时只在进入超时瞬间累加一次。
    uint32_t nrf_last_rx_time;          // 最近一次NRF有效收包时间，单位ms，来自HAL_GetTick()。

    uint32_t vision_rx_count;           // USB视觉底盘指令有效帧次数；protocol_handler解析到FUNC_CHASSIS_MOVE且校验通过后累加。
    uint32_t vision_timeout_count;      // USB视觉底盘指令超时次数；有效底盘帧超过100ms未更新时只在进入超时瞬间累加一次。
    uint32_t vision_checksum_error_count; // USB视觉协议校验失败次数；收到完整帧但checksum不匹配时累加。
    uint32_t vision_last_rx_time;       // 最近一次USB视觉底盘有效指令时间，单位ms，来自HAL_GetTick()。

    uint32_t usart2_rx_count;           // USART2/ctrl1宇树电机反馈接收完成次数；HAL_UART_RxCpltCallback中累加。
    uint32_t usart2_err_count;          // USART2错误回调次数；HAL_UART_ErrorCallback(&huart2)触发时累加，正常应不增长。
    uint32_t usart2_last_error;         // USART2最近一次HAL错误码；保存huart2.ErrorCode，用于判断噪声、溢出、DMA等错误类型。
    uint32_t usart3_rx_count;           // USART3/ctrl2宇树电机反馈接收完成次数；HAL_UART_RxCpltCallback中累加。
    uint32_t usart3_err_count;          // USART3错误回调次数；HAL_UART_ErrorCallback(&huart3)触发时累加，正常应不增长。
    uint32_t usart3_last_error;         // USART3最近一次HAL错误码；保存huart3.ErrorCode，用于排查ctrl2链路异常。

    uint32_t fdcan1_rx_count;           // FDCAN1接收帧次数；当前对应3508电机1/2反馈，正常使能后应持续增长。
    uint32_t fdcan1_err_count;          // FDCAN1接收FIFO读取失败次数；HAL_FDCAN_GetRxMessage失败时累加，正常应为0。
    uint32_t fdcan1_last_error;         // FDCAN1最近一次HAL错误码；保存hfdcan1.ErrorCode。
    uint32_t fdcan2_rx_count;           // FDCAN2接收帧次数；当前对应达妙机械臂电机反馈，正常使能后应持续增长。
    uint32_t fdcan2_err_count;          // FDCAN2接收FIFO读取失败次数；HAL_FDCAN_GetRxMessage失败时累加，正常应为0。
    uint32_t fdcan2_last_error;         // FDCAN2最近一次HAL错误码；保存hfdcan2.ErrorCode。
    uint32_t fdcan3_rx_count;           // FDCAN3接收帧次数；当前对应3508电机3/4反馈，可与fdcan1_rx_count对比判断3/4链路是否掉帧。
    uint32_t fdcan3_err_count;          // FDCAN3接收FIFO读取失败次数；HAL_FDCAN_GetRxMessage失败时累加，正常应为0。
    uint32_t fdcan3_last_error;         // FDCAN3最近一次HAL错误码；保存hfdcan3.ErrorCode。

    uint32_t loop_5ms_count;            // 5ms主调度循环执行次数；系统正常运行时约200次/秒增长。
    uint32_t loop_overrun_count;        // 5ms主调度超时次数；本工程暂以相邻调度间隔>6ms判定为一次overrun。
    uint32_t max_loop_dt_ms;            // 观测到的最大主循环间隔，单位ms；理想约5ms，明显变大说明主循环被阻塞或负载过高。
} RobotDiag_t;

extern volatile RobotDiag_t g_robot_diag;

void state_zero0(void);
void arm_knob_direct_control(void);
void Control_Init(QuadrupedGait* gait);
SystemCtrlMode_e Control_GetSystemMode(void);
void joystick_control(MotorController*ctrl1,MotorController*ctrl2,QuadrupedGait*gait,uint32_t startTime,LegAngles angles,uint32_t now);
void button_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t currentTime);
void AS01_rx(MotorController*ctrl1,MotorController*ctrl2,QuadrupedGait*gait,uint32_t startTime,LegAngles angles,uint32_t now);
void state_zero(uint32_t now, float current_kp);
#endif
