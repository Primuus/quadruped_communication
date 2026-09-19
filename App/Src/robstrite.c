//
// Created by 18737 on 26-4-2.
//

#include "robstrite.h"




uint16_t MotorCan_FloatToUint(float x, float x_min, float x_max, int bits)
{
    float span;
    float offset;
    uint32_t max_value;
    int result;

    span = x_max - x_min;
    offset = x_min;
    max_value = (uint32_t)((1U << bits) - 1U);

    if (x > x_max)
    {
        x = x_max;
    }
    else if (x < x_min)
    {
        x = x_min;
    }

    result = (int)((x - offset) * ((float)max_value) / span);

    if (result < 0)
    {
        result = 0;
    }
    else if ((uint32_t)result > max_value)
    {
        result = (int)max_value;
    }

    return (uint16_t)result;
}

void MotorCan_PackRobStriteMoveControl(AppCanTxMessage *message,
                                       uint8_t motor_id,
                                       float torque,
                                       float angle,
                                       float speed,
                                       float kp,
                                       float kd)
{
    uint16_t torque_u16;
    uint16_t angle_u16;
    uint16_t speed_u16;
    uint16_t kp_u16;
    uint16_t kd_u16;

    if (message == NULL)
    {
        return;
    }

    torque_u16 = MotorCan_FloatToUint(torque,
                                      MOTOR_CAN_ROBSTRITE_T_MIN,
                                      MOTOR_CAN_ROBSTRITE_T_MAX,
                                      16);

    angle_u16 = MotorCan_FloatToUint(angle,
                                     MOTOR_CAN_ROBSTRITE_P_MIN,
                                     MOTOR_CAN_ROBSTRITE_P_MAX,
                                     16);

    speed_u16 = MotorCan_FloatToUint(speed,
                                     MOTOR_CAN_ROBSTRITE_V_MIN,
                                     MOTOR_CAN_ROBSTRITE_V_MAX,
                                     16);

    kp_u16 = MotorCan_FloatToUint(kp,
                                  MOTOR_CAN_ROBSTRITE_KP_MIN,
                                  MOTOR_CAN_ROBSTRITE_KP_MAX,
                                  16);

    kd_u16 = MotorCan_FloatToUint(kd,
                                  MOTOR_CAN_ROBSTRITE_KD_MIN,
                                  MOTOR_CAN_ROBSTRITE_KD_MAX,
                                  16);

    message->ext_id = ((uint32_t)MOTOR_CAN_ROBSTRITE_COMM_TYPE_MOTION_CONTROL << 24) |
                      ((uint32_t)torque_u16 << 8) |
                      (uint32_t)motor_id;

    message->data[0] = (uint8_t)(angle_u16 >> 8);
    message->data[1] = (uint8_t)(angle_u16 & 0xFFU);
    message->data[2] = (uint8_t)(speed_u16 >> 8);
    message->data[3] = (uint8_t)(speed_u16 & 0xFFU);
    message->data[4] = (uint8_t)(kp_u16 >> 8);
    message->data[5] = (uint8_t)(kp_u16 & 0xFFU);
    message->data[6] = (uint8_t)(kd_u16 >> 8);
    message->data[7] = (uint8_t)(kd_u16 & 0xFFU);
}

void MotorCan_SendRobStriteTauRange(const AppTauCommand *tau_cmd,
                                    const uint8_t *motor_id_map,
                                    uint32_t tau_start_index,
                                    uint32_t motor_count,
                                    MotorCanSendFunc send_func)
{
    AppCanTxMessage message;
    uint32_t i;

    if ((tau_cmd == NULL) || (motor_id_map == NULL) || (send_func == NULL))
    {
        return;
    }

    if (tau_cmd->valid == 0U)
    {
        return;
    }

     for (i = 0U; i < motor_count; i++)
    {
        /* 纯力矩模式：angle/speed/kp/kd 全部为 0 */
        MotorCan_PackRobStriteMoveControl(&message,
                                          motor_id_map[i],
                                          (float)tau_cmd->tau[tau_start_index + i],
                                          0.0f,
                                          0.0f,
                                          0.0f,
                                          0.0f);

        send_func(&message);
    }
}


void MotorCan_PackRobStriteEnable(AppCanTxMessage *message,
                                  uint8_t motor_id,
                                  uint8_t master_id)
{
    uint32_t i;

    if (message == NULL)
    {
        return;
    }

    message->ext_id = ((uint32_t)MOTOR_CAN_ROBSTRITE_COMM_TYPE_MOTOR_ENABLE << 24) |
                      ((uint32_t)master_id << 8) |
                      (uint32_t)motor_id;

    for (i = 0U; i < APP_CAN_FRAME_DATA_SIZE; i++)
    {
        message->data[i] = 0U;
    }
}

void MotorCan_SendRobStriteEnableRange(const uint8_t *motor_id_map,
                                       uint32_t motor_count,
                                       uint8_t master_id,
                                       MotorCanSendFunc send_func)
{
    AppCanTxMessage message;
    uint32_t i;

    if ((motor_id_map == NULL) || (send_func == NULL))
    {
        return;
    }

    for (i = 0U; i < motor_count; i++)
    {
        MotorCan_PackRobStriteEnable(&message,
                                     motor_id_map[i],
                                     master_id);
        send_func(&message);
        osDelay(1U);
    }
}