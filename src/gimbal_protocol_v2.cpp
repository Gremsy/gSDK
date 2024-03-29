/*******************************************************************************
 * @file    gimbal_protocol_v2.cpp
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-21-2018
 * @brief   This file contains API for gimbal interface.
 *
 *  @Copyright (c) 2018 Gremsy
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "gimbal_protocol_v2.h"

Gimbal_Protocol_V2::Gimbal_Protocol_V2(Serial_Port *serial_port,
                                       const mavlink_system_t &system) :
    Gimbal_Protocol(serial_port, system) {}

/**
 * @brief  This function set gimbal mode
 * @param: type see control_mode_t
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Protocol_V2::set_gimbal_mode_sync(control_mode_t mode)
{
    if (!_is_init) {
        fprintf(stderr, "ERROR: could not set GIMBAL MODE, gimbal proto has not been initialized\n");
        return ERROR;
    }

    _control_mode = mode;
    return SUCCESS;
}

/**
 * @brief  This function reset gimbal with some mode
 * @param: type see gimbal_reset_mode_t
 * @ret: result
 */
Gimbal_Protocol_V2::result_t Gimbal_Protocol_V2::set_gimbal_reset_mode(gimbal_reset_mode_t reset_mode)
{
    float pitch = _attitude.pitch;
    float roll  = _attitude.roll;
    float yaw   = _attitude.yaw;

    switch (reset_mode) {
        case GIMBAL_RESET_MODE_YAW:
            yaw = 0.f;
            break;

        case GIMBAL_RESET_MODE_PITCH_AND_YAW:
            pitch = 0.f;
            roll  = 0.f;
            yaw   = 0.f;
            break;

        case GIMBAL_RESET_MODE_PITCH_DOWNWARD_UPWARD_AND_YAW:
            pitch = -90.f;
            roll  = 0.f;
            yaw   = 0.f;
            break;

        case GIMBAL_RESET_MODE_PITCH_DOWNWARD_UPWARD:
            pitch = -90.f;
            break;

        case GIMBAL_RESET_MODE_PITCH_MAPPING:
            pitch = -90.f;
            break;

        default:
            break;
    }
    /* Switch to follow */
    _control_mode = GIMBAL_FOLLOW_MODE;
    return set_gimbal_move_sync(pitch, roll, yaw, INPUT_ANGLE);
}

/**
 * @brief Set the gimbal move sync
 *
 * @param pitch control pitch value
 * @param roll control roll value
 * @param yaw control yaw value
 * @param mode see input_mode_t
 * @return result_t
 */
Gimbal_Protocol::result_t Gimbal_Protocol_V2::set_gimbal_move_sync(float pitch, float roll, float yaw,
        input_mode_t mode)
{
    if (_serial_port == nullptr) {
        fprintf(stderr, "ERROR: serial port not exist\n");
        throw 1;
    }

    if (!_is_init) {
        fprintf(stderr, "ERROR: could not MOVE GIMBAL, gimbal proto has not been initialized\n");
        return ERROR;
    }

    /* Pack message */
    mavlink_gimbal_device_set_attitude_t attitude = { 0 };
    attitude.target_system    = _gimbal.sysid;
    attitude.target_component = _gimbal.compid;
    attitude.flags = GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK |
                     ((_control_mode == GIMBAL_LOCK_MODE) ? GIMBAL_DEVICE_FLAGS_YAW_LOCK : 0);

    if (mode == INPUT_ANGLE) {
        /* Convert target to quaternion */
        mavlink_euler_to_quaternion(to_rad(roll), to_rad(pitch), to_rad(yaw), attitude.q);
        attitude.angular_velocity_x = NAN;
        attitude.angular_velocity_y = NAN;
        attitude.angular_velocity_z = NAN;

    } else {
        printf("%s %d %d\n", __func__, _system.sysid, _system.compid);

        attitude.angular_velocity_x = to_rad(roll);
        attitude.angular_velocity_y = to_rad(pitch);
        attitude.angular_velocity_z = to_rad(yaw);
        attitude.q[0] = NAN;
        attitude.q[1] = NAN;
        attitude.q[2] = NAN;
        attitude.q[3] = NAN;
    }

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = { 0 };
    mavlink_msg_gimbal_device_set_attitude_encode(_system.sysid, _system.compid, &message, &attitude);
    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // use_udp_send is a flag to internal use with PayloadSDK
    if(!use_udp_send){
        return (_serial_port->write_message(message) > 0) ? SUCCESS : ERROR;
    }
    else{
        if(__notifySendMessageCallback != NULL){
            __notifySendMessageCallback(message);
        }
        return SUCCESS;
    }

}
