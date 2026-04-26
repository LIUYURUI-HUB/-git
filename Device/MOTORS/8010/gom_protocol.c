#include <gom_protocol.h>
#include <Kalman.h>
#include <string.h>
#define SATURATE(_IN, _MIN, _MAX) \
	{                             \
		if ((_IN) <= (_MIN))      \
			(_IN) = (_MIN);       \
		else if ((_IN) >= (_MAX)) \
			(_IN) = (_MAX);       \
	}

static void UpdateMotorWFromposition(MotorData_t* motor_r, uint32_t current_time_ms)
{
    if (motor_r->last_update_ms == 0) {
        // 首次调用，仅初始化
    	motor_r->last_update_ms = current_time_ms;
    	motor_r->last_position = motor_r->Pos;
    	motor_r->last_true_W=0;
//    	motor_r->W = 0;
        return;
    }

    uint32_t dt_ms = current_time_ms - motor_r->last_update_ms;
    if (dt_ms == 0) {
        return; // 避免除零
    }
    if (current_time_ms >= motor_r->last_update_ms) {
        dt_ms = current_time_ms - motor_r->last_update_ms;
    } else {
        // 处理溢出情况
        dt_ms = (UINT32_MAX - motor_r->last_update_ms) + current_time_ms + 1;
    }
    if (dt_ms == 0 || dt_ms > 100) { // 假设最大合理间隔为100ms
           motor_r->last_update_ms = current_time_ms;
           motor_r->last_position = motor_r->Pos;
           return;
       }
    float cur_position=ema_filter(motor_r->Pos,motor_r->last_position, 0.9);
    motor_r->Pos=cur_position;
    double d_position = motor_r->Pos - motor_r->last_position; // 角度变化（度）

    double dt_min = dt_ms / 1000.0; // ms -> 分钟

    double w = d_position/  dt_min;

        float cur_w = ema_filter(w/6.33,motor_r->last_true_W, 0.9);
        motor_r->true_W = cur_w;
        motor_r->last_update_ms = current_time_ms;
        motor_r->last_true_W=cur_w;
        motor_r->last_position=motor_r->Pos;
}

void modify_data(MotorCmd_t *motor_s)
{
	motor_s->motor_send_data.head[0] = 0xFE;
	motor_s->motor_send_data.head[1] = 0xEE;

	SATURATE(motor_s->id, 0, 15);
	SATURATE(motor_s->mode, 0, 7);
	SATURATE(motor_s->K_P, 0.0f, 25.599f);
	SATURATE(motor_s->K_W, 0.0f, 25.599f);
	SATURATE(motor_s->T, -127.99f, 127.99f);
	SATURATE(motor_s->W, -804.00f, 804.00f);
	SATURATE(motor_s->Pos, -411774.0f, 411774.0f);

	motor_s->motor_send_data.mode.id = motor_s->id;
	motor_s->motor_send_data.mode.status = motor_s->mode;
	motor_s->motor_send_data.comd.k_pos = motor_s->K_P / 25.6f * 32768;
	motor_s->motor_send_data.comd.k_spd = motor_s->K_W / 25.6f * 32768;
	motor_s->motor_send_data.comd.pos_des = motor_s->Pos / 6.28318f * 32768;
	motor_s->motor_send_data.comd.spd_des = motor_s->W / 6.28318f * 256;
	motor_s->motor_send_data.comd.tor_des = motor_s->T * 256.0f;
	motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, sizeof(RIS_ControlData_t) - sizeof(motor_s->motor_send_data.CRC16));
}

/// @brief �����յ��Ķ�������ԭʼ����ת��Ϊ�����������
/// @param motor_r Ҫת���ĵ�������ṹ��
void extract_data(MotorData_t *motor_r)
{   uint32_t now = HAL_GetTick();
	if (motor_r->motor_recv_data.head[0] != 0xFD || motor_r->motor_recv_data.head[1] != 0xEE)
	{
		motor_r->correct = 0;
		return;
	}
	motor_r->calc_crc = crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, sizeof(RIS_MotorData_t) - sizeof(motor_r->motor_recv_data.CRC16));
	if (motor_r->motor_recv_data.CRC16 != motor_r->calc_crc)
	{
		memset(&motor_r->motor_recv_data, 0, sizeof(RIS_MotorData_t));
		motor_r->correct = 0;
		motor_r->bad_msg++;
		return;
	}
	else
	{
		motor_r->motor_id = motor_r->motor_recv_data.mode.id;
		motor_r->mode = motor_r->motor_recv_data.mode.status;
		motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
		motor_r->MError = motor_r->motor_recv_data.fbk.MError;
		motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed / 256) * 6.28318f;
		motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
		motor_r->Pos = 6.28318f * ((float)motor_r->motor_recv_data.fbk.pos) / 32768;
		motor_r->footForce = motor_r->motor_recv_data.fbk.force;
		motor_r->correct = 1;
		UpdateMotorWFromposition(motor_r, now);
		return;
	}
}

