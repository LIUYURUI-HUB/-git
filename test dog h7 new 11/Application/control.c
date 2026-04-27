#include "control.h"
#include <string.h> // 确保 memcpy 可用
#include <math.h>
#include "3508_driver.h"

extern RemoteData_t myPacket;
extern uint8_t rx_buffer[32];
uint8_t as01_status = 0;
extern uint8_t rx_data[32];
extern __attribute__((section(".RAM_D1")))__attribute__((section(".RAM_D1")))MotorController ctrl2;
extern MotorController ctrl1;

// ====== 【新增】姿态补偿相关外部声明 ======
extern float leg_y_offsets[4];
extern void state_zero_with_compensation(void);
extern float Chassis_Get_Leg_Offset(uint8_t leg_index);
// ==========================================

static float kp = 2.6f;
static GaitMode_e current_gait_mode = GAIT_MODE_TROT;
static uint8_t force_gait_update = 0;
static uint8_t is_single_action = 0;         // 单次动作标志位
static uint32_t single_action_end_time = 0;  // 单次动作结束时间
static GaitMode_e previous_gait_mode = GAIT_MODE_TROT; // 记录执行动作前的步态
static RobotState_e current_state = ROBOT_STATE_IDLE;
static uint8_t is_zeroing = 0;               // 标志位：是否正在执行复位插值
static uint32_t zeroing_start_time = 0;      // 记录复位动作开始的时间戳
static Currentpos zero_start_pos[4];         // 记录复位动作开始瞬间，4条腿的初始坐标
static const float ZERO_TARGET_X = 10.0f;     // 零点目标 X 坐标
static const float ZERO_TARGET_Y = 25.0f;    // 零点目标 Y 坐标
static const uint32_t ZEROING_DURATION_MS = 2500; // 复位动作耗时 (1000ms = 1秒)
static uint32_t current_zeroing_duration = 2500; // 新增：用于记录当前这次动作需要的总时长
static const uint32_t ZEROING_last_ms = 1000; // 复位动作耗时 (1000ms = 1秒)
static uint8_t is_adjusting_pose = 0;               // 标志位：是否正在执行高度微调
static uint32_t pose_adjust_start_time = 0;         // 记录微调动作开始的时间戳
static Currentpos pose_start_pos[4];                // 微调的起点
static Currentpos pose_target_pos[4];               // 微调的终点
static const uint32_t POSE_ADJUST_DURATION_MS = 300; // 微调耗时 (300ms)，让动作迅捷且平滑
static uint8_t is_pure_wheel_mode = 0;       // 【新增】纯轮模式标志位：0-轮腿联动步态，1-纯轮模式
static uint8_t is_first_zero = 0;

static inline void apply_current_gait(QuadrupedGait* gait) {
   float straight_period = 1.2f; // 周期 (秒)
   float straight_length = 8.5f; // 步长
   float straight_height = 10.0f; // 抬腿高度
   float turn_period = 0.8f;     // 转向周期 (通常更快一点)
   float turn_length = 3.0f;     // 转向步长 (建议改小，以轮子差速为主)
   float turn_height = 5.0f;     // 转向抬腿高度 (建议改高，防止腿绊住地面)

   // 根据当前状态选择应用的参数
   float p = straight_period;
   float l = straight_length;
   float h = straight_height;

   if (current_state == ROBOT_STATE_LEFT || current_state == ROBOT_STATE_RIGHT) {
       p = turn_period;
       l = turn_length;
       h = turn_height;
   }

   switch(current_gait_mode){
       case GAIT_MODE_WALK:
           init_quadruped_gait_walk(gait, 1.2f, l, h + 2.0f); // 举例：Walk也可以应用独立参数
           break;
       case GAIT_MODE_TROT:
           init_quadruped_gait_trot(gait, p, l, h);
           break;
       case GAIT_MODE_BOUND:
//          init_quadruped_gait_bound(gait, 0.6f, 10.849f, 25.0f);
           break;
       case GAIT_MODE_PRONK:
//          init_quadruped_gait_pronk(gait, 0.6f, 10.849f, 25.0f);
           break;
   }
}

/**
* @brief 启动零位插值：捕获当前位置并锁定状态
*/
void start_zero_interpolation(uint32_t now) {
   if (is_zeroing) return; // 正在插值中则忽略
   is_zeroing = 1;
   is_adjusting_pose = 0;  // 【重要】强制打断微调状态
   if (is_first_zero == 0) {
           current_zeroing_duration = ZEROING_DURATION_MS; // 2500ms
           is_first_zero = 1; // 标记以后不再是第一次了
       } else {
           current_zeroing_duration = ZEROING_last_ms;    // 1000ms
       }
   zeroing_start_time = now;
   zero_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
   zero_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
   zero_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
   zero_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);
}

/**
* @brief 零位插值执行：在 5ms 周期中平滑逼近坐标
*/
void state_zero(uint32_t now, float current_kp) {
   if (!is_zeroing) return;

   float current_target_x[4];
   float current_target_y[4];
   float progress = (float)(now - zeroing_start_time) / (float)current_zeroing_duration;
   if (progress >= 1.0f) {
       progress = 1.0f;
       is_zeroing = 0; // 进度到达 100%，结束插值
   }
   current_target_x[0] = zero_start_pos[0].X + (8 - zero_start_pos[0].X) * progress;
   current_target_y[0] = zero_start_pos[0].Y + (25 - zero_start_pos[0].Y) * progress;
   for (int i = 1; i < 4; i++) {
       current_target_x[i] = zero_start_pos[i].X + (ZERO_TARGET_X - zero_start_pos[i].X) * progress;
       current_target_y[i] = zero_start_pos[i].Y + (ZERO_TARGET_Y - zero_start_pos[i].Y) * progress;
   }
   // 【核心修改】：统一给4条腿加上实时姿态补偿
      for (int i = 0; i < 4; i++) {
          current_target_y[i] += Chassis_Get_Leg_Offset(i);
      }

   LegAngles angles1 = InverseKinematics(current_target_x[0], current_target_y[0], &ctrl2, 1, 2, 0.3, 0.1);
   MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles1.theta2, current_kp, 0.1);
   MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles1.theta1, current_kp, 0.1);

   LegAngles angles2 = InverseKinematics(current_target_x[1], current_target_y[1], &ctrl2, 7, 8, 0.3, 0.1);
   MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles2.theta2, current_kp, 0.1);
   MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles2.theta1, current_kp, 0.1);

   LegAngles angles3 = InverseKinematics(current_target_x[2], current_target_y[2], &ctrl1, 6, 5, 0.3, 0.1);
   MotorController_SetCommand(&ctrl1, 5, 1, 0.0, 0.0f, angles3.theta2, current_kp, 0.1);
   MotorController_SetCommand(&ctrl1, 6, 1, 0.0, 0.0f, angles3.theta1, current_kp, 0.1);

   LegAngles angles4 = InverseKinematics(current_target_x[3], current_target_y[3], &ctrl1, 3, 4, 0.3, 0.1);
   MotorController_SetCommand(&ctrl1, 4, 1, 0.0, 0.0f, angles4.theta2, current_kp, 0.1);
   MotorController_SetCommand(&ctrl1, 3, 1, 0.0, 0.0f, angles4.theta1, current_kp, 0.1);
}

void start_pose_adjustment(uint32_t now, float delta_x, float delta_y) {
   if (is_adjusting_pose) return;
   is_adjusting_pose = 1;
   is_zeroing = 0;        // 【重要】强制打断归零插值状态
   pose_adjust_start_time = now;
   pose_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
   pose_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
   pose_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
   pose_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);

   // 计算终点目标位置
   for (int i = 0; i < 4; i++) {
       pose_target_pos[i].X = pose_start_pos[i].X + delta_x;
       pose_target_pos[i].Y = pose_start_pos[i].Y + delta_y;
   }
}

/**
* @brief 姿态微调执行：在 5ms 周期中平滑逼近微调后的坐标
*/
void update_pose_adjustment(uint32_t now, float current_kp) {
   if (!is_adjusting_pose) return;

   float progress = (float)(now - pose_adjust_start_time) / (float)POSE_ADJUST_DURATION_MS;
   if (progress >= 1.0f) {
       progress = 1.0f;
       is_adjusting_pose = 0;
   }

   float current_target_x[4];
   float current_target_y[4];

   for (int i = 0; i < 4; i++) {
       current_target_x[i] = pose_start_pos[i].X + (pose_target_pos[i].X - pose_start_pos[i].X) * progress;
       current_target_y[i] = pose_start_pos[i].Y + (pose_target_pos[i].Y - pose_start_pos[i].Y) * progress;

       // 2. 【核心修改】：加上底盘姿态补偿量
       current_target_y[i] += Chassis_Get_Leg_Offset(i);
   }

   LegAngles angles1 = InverseKinematics(current_target_x[0], current_target_y[0], &ctrl2, 1, 2, 0.5, 0.1);
   MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles1.theta2, current_kp, 0.1);
   MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles1.theta1, current_kp, 0.1);

   LegAngles angles2 = InverseKinematics(current_target_x[1], current_target_y[1], &ctrl2, 7, 8, 0.5, 0.1);
   MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles2.theta2, current_kp, 0.1);
   MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles2.theta1, current_kp, 0.1);

   LegAngles angles3 = InverseKinematics(current_target_x[2], current_target_y[2], &ctrl1, 3, 4, 0.5, 0.1);
   MotorController_SetCommand(&ctrl1, 4, 1, 0.0, 0.0f, angles3.theta2, current_kp, 0.1);
   MotorController_SetCommand(&ctrl1, 3, 1, 0.0, 0.0f, angles3.theta1, current_kp, 0.1);

   LegAngles angles4 = InverseKinematics(current_target_x[3], current_target_y[3], &ctrl1, 6, 5, 0.5, 0.1);
   MotorController_SetCommand(&ctrl1, 5, 1, 0.0, 0.0f, angles4.theta2, current_kp, 0.1);
   MotorController_SetCommand(&ctrl1, 6, 1, 0.0, 0.0f, angles4.theta1, current_kp, 0.1);
}

/**
* @brief 摇杆控制处理
*/
void joystick_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
   RobotState_e target_state = ROBOT_STATE_IDLE;
   float currentTimeSec = now / 1000.0f;
   static uint32_t last_calc_time = 0;

   if (is_single_action) {
       if (now >= single_action_end_time) {
           is_single_action = 0;
           current_gait_mode = previous_gait_mode;
           apply_current_gait(gait);
           current_state = ROBOT_STATE_IDLE;
           start_zero_interpolation(now);          // 触发跳跃后的平滑复位
       } else {
           if (now - last_calc_time >= 5) {
               last_calc_time = now;
               // 【姿态补偿融入点 1】：单次动作执行时，代入静态补偿数组
               angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1, leg_y_offsets[0]);
               angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, kp, 0.1, leg_y_offsets[1]);
               angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1, leg_y_offsets[2]);
               angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1, leg_y_offsets[3]);

               // 单次动作时保持 3508 轮子停止
               for (int i = 0; i < 4; i++) {
                   Motors[i].target_speed = 0.0f;
                   PID_Calc_Speed(i);
               }
           }
       }
       return;
   }

   if (myPacket.joy_lx > 60) {
       target_state = ROBOT_STATE_BACKWARD;
   } else if (myPacket.joy_lx < 40) {
       target_state = ROBOT_STATE_FORWARD;
   } else if (myPacket.joy_rx > 60) {
       target_state = ROBOT_STATE_LEFT;
   } else if (myPacket.joy_rx < 40) {
       target_state = ROBOT_STATE_RIGHT;
   } else {
       target_state = ROBOT_STATE_IDLE;
   }

   // 状态切换检测
   if (target_state != current_state) {
       current_state = target_state;
       force_gait_update = 0;
       switch (current_state) {
           case ROBOT_STATE_FORWARD:
               if (!is_pure_wheel_mode) {
                   apply_current_gait(gait);
                   calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
                   calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                   calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
                   calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
                   start_quadruped_gait(gait, currentTimeSec);
               }
               break;
           case ROBOT_STATE_BACKWARD:
               if (!is_pure_wheel_mode) {
                   apply_current_gait(gait);
                   calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
                   calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
                   calibrate_leg_base_position(gait, 2, ctrl1,6,5,0);
                   calibrate_leg_base_position(gait, 3, ctrl1,3,4,0);
                   start_quadruped_gait(gait, currentTimeSec);
               }
               break;
           case ROBOT_STATE_LEFT:
               if (!is_pure_wheel_mode) {
                   apply_current_gait(gait);
                   calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
                   calibrate_leg_base_position(gait, 2, ctrl1,3,4,1);
                   calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                   calibrate_leg_base_position(gait, 3, ctrl1,6,5,0);
                   start_quadruped_gait(gait, currentTimeSec);
               }
               break;
           case ROBOT_STATE_RIGHT:
               if (!is_pure_wheel_mode) {
                   apply_current_gait(gait);
                   calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
                   calibrate_leg_base_position(gait, 2, ctrl1,3,4,0);
                   calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
                   calibrate_leg_base_position(gait, 3, ctrl1,6,5,1);
                   start_quadruped_gait(gait, currentTimeSec);
               }
               break;
           case ROBOT_STATE_IDLE:
               start_zero_interpolation(now);
               break;
       }
   }

   if (now - last_calc_time >= 5) {
       last_calc_time = now;
       if (current_state != ROBOT_STATE_IDLE) {

           if (!is_pure_wheel_mode) {
               if (current_state == ROBOT_STATE_FORWARD || current_state == ROBOT_STATE_BACKWARD) {
                   // 【姿态补偿融入点 2】：正常行进时的动态补偿计算
                   angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1, Chassis_Get_Leg_Offset(0));
                   angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, kp, 0.1, Chassis_Get_Leg_Offset(2));
                   angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1, Chassis_Get_Leg_Offset(1));
                   angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1, Chassis_Get_Leg_Offset(3));
               } else {
                   // 转向时的补偿
                   angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, 2.6, 0.1, Chassis_Get_Leg_Offset(0));
                   angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, 2.6, 0.1, Chassis_Get_Leg_Offset(2));
                   angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, 2.6, 0.1, Chassis_Get_Leg_Offset(1));
                   angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, 2.6, 0.1, Chassis_Get_Leg_Offset(3));
               }
           } else {
               // 纯轮模式下前进后退时，保持腿部处于归零后的站立姿态
               if (is_zeroing) {
                   state_zero(now, kp);
               } else if (is_adjusting_pose) {
                   update_pose_adjustment(now, kp);
               } else {
                   // 【姿态补偿融入点 3】：纯轮移动状态下的连续姿态补偿保护
                   state_zero_with_compensation();
               }
           }
          float linear_ratio = 0.0f; // 前后比例：前进为正，后退为负
          if (myPacket.joy_lx < 40) {
                           linear_ratio = (40.0f - myPacket.joy_lx) / 40.0f; // 摇杆越靠近 0，比例越接近 1.0
             } else if (myPacket.joy_lx > 60) {
                           linear_ratio = (60.0f - myPacket.joy_lx) / 40.0f; // 摇杆越靠近 100，比例越接近 -1.0
             }

          float turn_ratio = 0.0f; // 转向比例：右转为正，左转为负
          if (myPacket.joy_rx < 40) {
                           turn_ratio = (40.0f - myPacket.joy_rx) / 40.0f;
              } else if (myPacket.joy_rx > 60) {
                           turn_ratio = (60.0f - myPacket.joy_rx) / 40.0f;
                }
           float max_base_rpm = is_pure_wheel_mode ? 2400.0f : 1800.0f;
           float max_turn_rpm = is_pure_wheel_mode ? 1800.0f : 1500.0f; // 注意这里用正数表示转向幅度
           float wheel_target_rpm_01 = (linear_ratio * max_base_rpm) + (turn_ratio * max_turn_rpm);
           float wheel_target_rpm_23 = -(linear_ratio * max_base_rpm) + (turn_ratio * max_turn_rpm);

           for (int i = 0; i < 2; i++) {
               Motors[i].target_speed = wheel_target_rpm_01;
               PID_Calc_Speed(i);
           }
           for (int i = 2; i < 4; i++) {
               Motors[i].target_speed = wheel_target_rpm_23;
               PID_Calc_Speed(i);
           }
           // ==========================================

       } else {
           if (is_zeroing) {
               state_zero(now, kp);
           } else if (is_adjusting_pose) {
               update_pose_adjustment(now, kp);
           } else {
               // 【姿态补偿融入点 4】：静止站立时，插值彻底结束后的连续姿态补偿保护
               state_zero_with_compensation();
           }
           for (int i = 0; i < 4; i++) {
               Motors[i].target_speed = 0.0f;
               PID_Calc_Speed(i);
           }
       }
   }
}

/**
* @brief 按键控制逻辑
*/
#define DEBOUNCE_DELAY_MS 30
#define NUM_BUTTONS 8

void button_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t currentTime) {
   static uint8_t stable_button_states[NUM_BUTTONS] = {0};
   static uint8_t last_button_readings[NUM_BUTTONS] = {0};
   static uint32_t last_debounce_times[NUM_BUTTONS] = {0};

	float dy = 1.0f;
   float dx = 0.0f;

   // 消抖与按键处理
   for (int i = 0; i < NUM_BUTTONS; i++) {
       uint8_t current_reading = myPacket.button[i];

       if (current_reading != last_button_readings[i]) {
           last_debounce_times[i] = currentTime;
       }

       if ((currentTime - last_debounce_times[i]) > DEBOUNCE_DELAY_MS) {
           if (current_reading != stable_button_states[i]) {
               stable_button_states[i] = current_reading;
               if (stable_button_states[i] == 1) {
                   switch (i) {
                       case 0:
                           // 归零
                           start_zero_interpolation(currentTime);
                           break;

                       case 1:
                           // 升高底盘 (Y+dy, X-dx) - 触发微调插值
                           start_pose_adjustment(currentTime, dx, dy);
                           break;

                       case 2:
                           // 降低底盘 (Y-dy, X-dx) - 触发微调插值
                           start_pose_adjustment(currentTime, -dx, -dy);
                           break;

                       case 3:
                           is_pure_wheel_mode = !is_pure_wheel_mode;
                           if (is_pure_wheel_mode) {
                           start_zero_interpolation(currentTime);
                           } else {
                            current_gait_mode = GAIT_MODE_TROT;
                            force_gait_update = 1;
                            }
                           break;

                       case 4:
                           if (current_gait_mode != GAIT_MODE_TROT) {
                               current_gait_mode = GAIT_MODE_TROT;
                               force_gait_update = 1;
                           }

                           break;

                       case 5:
                       	if (current_gait_mode != GAIT_MODE_WALK) {
                               current_gait_mode = GAIT_MODE_WALK;
                               force_gait_update = 1;
                           }

                           break;
                       case 6: // 单次执行 BOUND
                           break;

                       case 7: // 单次执行 PRONK
                           break;

                       default:
                           break;
                   }
               }
           }
       }
       last_button_readings[i] = current_reading;
   }
}
#define REMOTE_TIMEOUT_MS 300 // 定义通信超时时间为 300 毫秒

/**
* @brief 接收数据并分发控制 (包含超时失联保护)
*/
void AS01_rx(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
   static uint32_t last_rx_time = 0; // 静态变量：记录上一次成功接收到数据的时间戳

   // 1. 尝试接收数据
   if (NRF24L01_RxPacket(rx_buffer) == 0) {
       memcpy(&myPacket, rx_buffer, sizeof(RemoteData_t));
       HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
       last_rx_time = now; // 成功接收，更新时间戳
   }
   if ((now - last_rx_time) > REMOTE_TIMEOUT_MS) {
       // 强制将摇杆数据归至中位 (触发 ROBOT_STATE_IDLE)
       myPacket.joy_lx = 50;
       myPacket.joy_rx = 50;
       myPacket.joy_ly = 50;
       myPacket.joy_ry = 50;
       memset(myPacket.button, 0, sizeof(myPacket.button));
   } else {
   }

   // 3. 执行控制逻辑
   joystick_control(ctrl1, ctrl2, gait, startTime, angles, now);
   button_control(ctrl1, ctrl2, gait, now);
}
