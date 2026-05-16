#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "../../BSP/MOTORS/8010/motor_controller.h"
#include "math.h"
//float sqrt_float(float x) {
//    return sqrtf(x);
//}
//
//double sqrt_double(double x) {
//    return sqrt(x);
//}

typedef struct {
    float theta1;
    float theta2;
} LegAngles;
typedef struct{
	float X ;
	float Y ;
}Currentpos;

//uint8_t InverseKinematics(float X, float Y, LegAngles *angles);
LegAngles InverseKinematics(float X, float Y ,MotorController* ctrl,int hip_id,int calf_id, float kp, float kd);
Currentpos ForwardKinematics(MotorController *ctrl,int hip_id, int knee_id);

#endif // __KINEMATICS_H__
