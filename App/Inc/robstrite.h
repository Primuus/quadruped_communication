//
// Created by 18737 on 26-4-2.
//

#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#define CAN_ROBSTRITE_MASTER_ID  0x00U
#include <stdint.h>
#include "app_tasks.h"
#define MOTOR_CAN_ROBSTRITE_COMM_TYPE_MOTION_CONTROL   0x01U

#define MOTOR_CAN_ROBSTRITE_P_MIN   (-12.5f)
#define MOTOR_CAN_ROBSTRITE_P_MAX   ( 12.5f)
#define MOTOR_CAN_ROBSTRITE_V_MIN   (-44.0f)
#define MOTOR_CAN_ROBSTRITE_V_MAX   ( 44.0f)
#define MOTOR_CAN_ROBSTRITE_KP_MIN  (  0.0f)
#define MOTOR_CAN_ROBSTRITE_KP_MAX  (500.0f)
#define MOTOR_CAN_ROBSTRITE_KD_MIN  (  0.0f)
#define MOTOR_CAN_ROBSTRITE_KD_MAX  (  5.0f)
#define MOTOR_CAN_ROBSTRITE_T_MIN   (-17.0f)
#define MOTOR_CAN_ROBSTRITE_T_MAX   ( 17.0f)
#define MOTOR_CAN_ROBSTRITE_COMM_TYPE_MOTOR_ENABLE   0x03U


typedef void (*MotorCanSendFunc)(const AppCanTxMessage *message);

uint16_t MotorCan_FloatToUint(float x, float x_min, float x_max, int bits);

void MotorCan_PackRobStriteMoveControl(AppCanTxMessage *message,
                                       uint8_t motor_id,
                                       float torque,
                                       float angle,
                                       float speed,
                                       float kp,
                                       float kd);
void MotorCan_SendRobStriteTauRange(const AppTauCommand *tau_cmd,
                                        const uint8_t *motor_id_map,
                                        uint32_t tau_start_index,
                                        uint32_t motor_count,
                                        MotorCanSendFunc send_func);

void MotorCan_PackRobStriteEnable(AppCanTxMessage *message,
                                  uint8_t motor_id,
                                  uint8_t master_id);

void MotorCan_SendRobStriteEnableRange(const uint8_t *motor_id_map,
                                       uint32_t motor_count,
                                       uint8_t master_id,
                                       MotorCanSendFunc send_func);

#endif
