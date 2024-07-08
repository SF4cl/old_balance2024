//=====================================================================================================
// DMMotor.c
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
//
//
//=====================================================================================================
#include "DMMotor.h"

#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "user_lib.h"

static DM_Motor_t* dm_motor[MAX_DM_MOTOR_NUM];
static uint8_t id_cnt;  // 记录达妙电机数量

static uint8_t motor_send_buffer[8];

static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
static int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void dmMotorCallback(Can_Device_t* can_device) {
    if (can_device == NULL) return;

    DM_Motor_t* motor = can_device->parent;
    dmMotorInfoUpdate(motor, can_device->rx_buff);

    if (motor->motorCallback != NULL) {
        motor->motorCallback(motor);
    }
    motor->motor_common.statu = STATE_ONLINE;
}

DM_Motor_t* dmMotorAdd(DM_Motor_Register_t* reg) {
    if (reg == NULL) return NULL;

    if (id_cnt > MAX_DM_MOTOR_NUM) {
        Error_Handler();  // 电机太多
    }

    Can_Register_t can_reg = {};
    DM_Motor_t* motor = (DM_Motor_t*)malloc(sizeof(DM_Motor_t));
    memset(motor, 0, sizeof(DM_Motor_t));

    can_reg.can_handle = reg->can_handle;
    can_reg.tx_dlc = 8;
    can_reg.can_device_callback = dmMotorCallback;
    can_reg.can_device_offline_callback = reg->motor_register_common.motorOfflineCallback;
    can_reg.tx_id = reg->tx_id;
    can_reg.rx_id = reg->rx_id;
    can_reg.parent = motor;
    can_reg.offline_threshold = MOTOR_OFFLINE_THRESHOLD;

    motor->motor_common.motor_type = reg->motor_register_common.motor_type;
    motor->p_max = reg->p_max;
    motor->v_max = reg->v_max;
    motor->t_max = reg->t_max;
    motor->kp_max = reg->kp_max;
    motor->kd_max = reg->kd_max;
    motor->kp_min = reg->kp_min;
    motor->kd_min = reg->kd_min;
    motor->can_info = canDeviceRegister(&can_reg);

    motor->motor_common.statu = STATE_ONLINE;
    dm_motor[id_cnt++] = motor;

    dmMotorEnable(motor);

    return motor;
}

Return_t dmMotorSendMessage(DM_Motor_t* motor) {
    if (motor->motor_common.statu == STATE_OFFLINE) {
        memset(&motor->command_interfaces, 0, sizeof(motor->command_interfaces));
        return dmMotorEnable(motor);
    }

    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    float_constrain(motor->command_interfaces.pos, -motor->p_max, motor->p_max);
    float_constrain(motor->command_interfaces.vel, -motor->v_max, motor->v_max);
    float_constrain(motor->command_interfaces.t, -motor->p_max, motor->p_max);

    pos_tmp = float_to_uint(motor->command_interfaces.pos, -motor->p_max, motor->p_max, 16);
    vel_tmp = float_to_uint(motor->command_interfaces.vel, -motor->v_max, motor->v_max, 12);
    kp_tmp = float_to_uint(motor->command_interfaces.kp, motor->kp_min, motor->kp_max, 12);
    kd_tmp = float_to_uint(motor->command_interfaces.kd, motor->kd_min, motor->kd_max, 12);
    tor_tmp = float_to_uint(motor->command_interfaces.t, -motor->p_max, motor->p_max, 12);

    motor_send_buffer[0] = (pos_tmp >> 8);
    motor_send_buffer[1] = pos_tmp;
    motor_send_buffer[2] = (vel_tmp >> 4);
    motor_send_buffer[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    motor_send_buffer[4] = kp_tmp;
    motor_send_buffer[5] = (kd_tmp >> 4);
    motor_send_buffer[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    motor_send_buffer[7] = tor_tmp;

    return canSendMessage(motor->can_info, motor_send_buffer);
}

Return_t dmMotorSendAll() {
    uint8_t i = 0;
    Return_t ret = RETURN_SUCCESS;
    for (i = 0; i < id_cnt; i++) {
        if (dmMotorSendMessage(dm_motor[i]) == RETURN_ERROR) {
            ret = RETURN_ERROR;
        }
    }

    return ret;
}

void dmMotorInfoUpdate(DM_Motor_t* motor, uint8_t* data) {
    motor->state_interfaces.id = data[0] & 0xf;
    motor->state_interfaces.error = data[0] >> 4;

    motor->state_interfaces.pos = ((uint16_t)data[1] << 8) | data[2];
    motor->state_interfaces.position = uint_to_float(motor->state_interfaces.pos, -motor->p_max, motor->p_max, 16);

    motor->state_interfaces.vel = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    motor->state_interfaces.velocity = uint_to_float(motor->state_interfaces.vel, -motor->v_max, motor->v_max, 12);

    motor->state_interfaces.t = ((uint16_t)(data[4] & 0xf) << 8) | (data[5]);
    motor->state_interfaces.torque = uint_to_float(motor->state_interfaces.t, -motor->t_max, motor->t_max, 12);

    motor->state_interfaces.t_mos = data[6];
    motor->state_interfaces.t_rotor = data[7];
}

Return_t dmMotorEnable(DM_Motor_t* motor) {
    motor_send_buffer[0] = 0xFF;
    motor_send_buffer[1] = 0xFF;
    motor_send_buffer[2] = 0XFF;
    motor_send_buffer[3] = 0xFF;
    motor_send_buffer[4] = 0xFF;
    motor_send_buffer[5] = 0xFF;
    motor_send_buffer[6] = 0xFF;
    motor_send_buffer[7] = 0xFC;

    return canSendMessage(motor->can_info, motor_send_buffer);
}
void dmMotorDisable(DM_Motor_t* motor) {
    motor_send_buffer[0] = 0xFF;
    motor_send_buffer[1] = 0xFF;
    motor_send_buffer[2] = 0XFF;
    motor_send_buffer[3] = 0xFF;
    motor_send_buffer[4] = 0xFF;
    motor_send_buffer[5] = 0xFF;
    motor_send_buffer[6] = 0xFF;
    motor_send_buffer[7] = 0xFD;

    canSendMessage(motor->can_info, motor_send_buffer);
}
void dmMotorSetZero(DM_Motor_t* motor) {
    motor_send_buffer[0] = 0xFF;
    motor_send_buffer[1] = 0xFF;
    motor_send_buffer[2] = 0XFF;
    motor_send_buffer[3] = 0xFF;
    motor_send_buffer[4] = 0xFF;
    motor_send_buffer[5] = 0xFF;
    motor_send_buffer[6] = 0xFF;
    motor_send_buffer[7] = 0xFE;

    canSendMessage(motor->can_info, motor_send_buffer);
}
void dmMotorClearError(DM_Motor_t* motor) {
    motor_send_buffer[0] = 0xFF;
    motor_send_buffer[1] = 0xFF;
    motor_send_buffer[2] = 0XFF;
    motor_send_buffer[3] = 0xFF;
    motor_send_buffer[4] = 0xFF;
    motor_send_buffer[5] = 0xFF;
    motor_send_buffer[6] = 0xFF;
    motor_send_buffer[7] = 0xFB;

    canSendMessage(motor->can_info, motor_send_buffer);
}