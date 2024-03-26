/*******************************************************************************
 * @file    gimbal_protocol.cpp
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

#include <errno.h>

#include "gimbal_protocol.h"

Gimbal_Protocol::Gimbal_Protocol(Serial_Port *serial_port,
                                 const mavlink_system_t &system) :
    _system(system),
    _attitude()
{
    _serial_port = serial_port;
    pthread_mutex_init(&_mutex, NULL);
    pthread_cond_init(&_condition, NULL);
}

Gimbal_Protocol::~Gimbal_Protocol()
{
    pthread_mutex_destroy(&_mutex);
    pthread_cond_destroy(&_condition);
}

void 
Gimbal_Protocol::
regSendMessageCallback(send_message_callback_t func){
    __notifySendMessageCallback = func;
}

/**
 * @brief Function init protocol
 *
 * @param gimbal
 */
void Gimbal_Protocol::initialize(const mavlink_system_t &gimbal)
{
    _gimbal.sysid  = gimbal.sysid;
    _gimbal.compid = gimbal.compid;
    _is_init = true;
}

/**
 * @brief Function send commad long to gimbal
 *
 * @param command command ID
 * @param param param1 -> param7
 * @return result_t SUCCESS if send command successfully
 */
Gimbal_Protocol::result_t Gimbal_Protocol::send_command_long(uint16_t command, const float param[7])
{
    if (_serial_port == nullptr) {
        fprintf(stderr, "ERROR: serial port not exist\n");
        throw 1;
    }

    if (!_is_init) {
        fprintf(stderr, "ERROR: could not send COMMAND_LONG, gimbal proto has not been initialized\n");
        return ERROR;
    }

    // Prepare command for off-board mode
    mavlink_command_long_t cmd = { 0 };
    cmd.target_system    = _gimbal.sysid;
    cmd.target_component = _gimbal.compid;
    cmd.command          = command;
    memcpy(&cmd.param1, param, 7 * sizeof(float));
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = { 0 };
    mavlink_msg_command_long_encode(_system.sysid, _system.compid, &message, &cmd);
    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    return (_serial_port->write_message(message) > 0) ? SUCCESS : ERROR;
}

/**
 * @brief Function send commad long to gimbal and block to wait for ack response
 *
 * @param command command ID
 * @param param param1 -> param7
 * @return result_t ack response from gimbal
 */
Gimbal_Protocol::result_t Gimbal_Protocol::send_command_long_sync(uint16_t command, const float param[7])
{
    if (send_command_long(command, param) == SUCCESS) {
        struct timespec time_wait;
        int rt = 0;
        result_t result = UNKNOWN;
        // get current time
        clock_gettime(CLOCK_REALTIME, &time_wait);
        // Wait for 1 sec timeout
        time_wait.tv_sec += 1;
        // Lock mutex
        pthread_mutex_lock(&_mutex);
        // Wait for ack reply
        rt = pthread_cond_timedwait(&_condition, &_mutex, &time_wait);

        // check result
        if (rt == ETIMEDOUT) {
            result = TIMEOUT;

        } else {
            if (_ack.command == command) {
                result = from_mav_result((MAV_RESULT)_ack.result);
            }
        }

        // Unlock mutex
        pthread_mutex_unlock(&_mutex);
        return result;
    }

    return ERROR;
}

/**
 * @brief callback for command ack
 * @details called from serial read message
 * @param message
 */
void Gimbal_Protocol::command_ack_callback(const mavlink_message_t &message)
{
    if (!_is_init) {
        return;
    }

    // Reset ack
    memset(&_ack, 0, sizeof(mavlink_command_ack_t));
    mavlink_msg_command_ack_decode(&message, &_ack);

    if (_ack.target_system == _system.sysid && _ack.target_component == _system.compid) {
        // Lock mutex
        pthread_mutex_lock(&_mutex);
        pthread_cond_signal(&_condition);
        // Unlock mutex
        pthread_mutex_unlock(&_mutex);
    }
}

/**
 * @brief convert from mav result
 *
 * @param res
 * @return result_t
 */
Gimbal_Protocol::result_t Gimbal_Protocol::from_mav_result(MAV_RESULT res)
{
    switch (res) {
        case MAV_RESULT_ACCEPTED:
            return SUCCESS;
            break;

        case MAV_RESULT_DENIED:
            return DENIED;
            break;

        case MAV_RESULT_IN_PROGRESS:
            return SUCCESS;
            break;

        default:
            return UNKNOWN;
            break;
    }

    return UNKNOWN;
}

/**
 * @brief Update gimbal attitude for gimbal protocol instance
 *
 * @param pitch
 * @param roll
 * @param yaw
 */
void Gimbal_Protocol::update_attitude(float pitch, float roll, float yaw)
{
    _attitude.pitch = pitch;
    _attitude.roll  = roll;
    _attitude.yaw   = yaw;
}

/**
 * @brief Update gimbal attitude for gimbal protocol instance
 *
 * @param q attitude quaternion
 */
void Gimbal_Protocol::update_attitude(const float q[4])
{
    mavlink_quaternion_to_euler(q, &_attitude.roll, &_attitude.pitch, &_attitude.yaw);
    (void)_attitude.to_deg();
}
