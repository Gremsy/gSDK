/*******************************************************************************
 * @file    gimbal_protocol_v1.cpp
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

#include "gimbal_protocol_v1.h"

using namespace GSDK;

Gimbal_Protocol_V1::Gimbal_Protocol_V1(HAL::gSDK_Serial_Manager *serial, const mavlink_system_t &system) :
    Gimbal_Protocol(serial, system) {}

/**
 * @brief  This function set gimbal mode
 * @param: type see control_mode_t
 * @ret: result
 */
result_t Gimbal_Protocol_V1::set_gimbal_mode_sync(control_mode_t mode)
{
    const float param[7] = {
        0,
        0,
        0,
        0,
        0,
        0,
        (float)mode
    };
    return send_command_long_sync(MAV_CMD_USER_2, param);
}

/**
 * @brief  This function reset gimbal with some mode
 * @param: type see gimbal_reset_mode_t
 * @ret: result
 */
result_t Gimbal_Protocol_V1::set_gimbal_reset_mode(gimbal_reset_mode_t reset_mode)
{
    const float param[7] = {
        0,
        0,
        0,
        0,
        0,
        (float)reset_mode,
        (float)GIMBAL_RESET_MODE
    };
    return send_command_long_sync(MAV_CMD_USER_2, param);
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
result_t Gimbal_Protocol_V1::set_gimbal_move_sync(float pitch, float roll, float yaw,
        input_mode_t mode)
{
    const float param[7] = {
        pitch,
        roll,
        yaw,
        0,
        0,
        (float)mode,
        (float)MAV_MOUNT_MODE_MAVLINK_TARGETING
    };
    return send_command_long_sync(MAV_CMD_DO_MOUNT_CONTROL, param);
}
