/*******************************************************************************
 * @file    gimbal_protocol.h
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-21-2018
 * @brief   This file contains API for gimbal interface
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

#ifndef GIMBAL_PROTOCOL_H_
#define GIMBAL_PROTOCOL_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <stdint.h>

#include "serial_port.h"

#include <ardupilotmega/mavlink.h>

// ----------------------------------------------------------------------------------
//   Gimbal Protocol Class
// ----------------------------------------------------------------------------------
/*
 * Gimbal Protocol Class
 *
 * This class implement base class for Gimbal Protocol
 */
class Gimbal_Protocol
{
public:
    /**
     * @brief Enum described function return status
     * 
     */
    enum result_t {
        SUCCESS,
        ERROR,
        DENIED,
        UNKNOWN,
        TIMEOUT,
    };

    /**
     * @brief Control gimbal Mode
     * @details Mode LOCK/FOLLOW/RESET MODE
     */
    enum control_mode_t {
        GIMBAL_OFF         = 0x00,
        GIMBAL_LOCK_MODE   = 0x01,
        GIMBAL_FOLLOW_MODE = 0x02,
        GIMBAL_RESET_MODE  = 0x04
    };

    /**
     * @brief Reset mode of gimbal.
     */
    enum gimbal_reset_mode_t {
        /*! Only reset yaw axis of gimbal. Reset angle of yaw axis to the sum of yaw axis angle of aircraft and fine tune angle
        * of yaw axis of gimbal. */
            GIMBAL_RESET_MODE_YAW = 1,
        /*! Reset yaw axis and pitch axis of gimbal. Reset angle of yaw axis to sum of yaw axis angle of aircraft and fine tune 
        * angle of yaw axis of gimbal, and reset pitch axis angle to the fine tune angle. */
            GIMBAL_RESET_MODE_PITCH_AND_YAW = 3,
        /*! Reset yaw axis and pitch axis of gimbal. Reset angle of yaw axis to sum of yaw axis angle of aircraft and fine tune
        * angle of yaw axis of gimbal, and reset pitch axis angle to sum of -90 degree and fine tune angle if gimbal
        * downward, sum of 90 degree and fine tune angle if upward. */
            GIMBAL_RESET_MODE_PITCH_DOWNWARD_UPWARD_AND_YAW = 11,
        /*! Reset pitch axis of gimbal. Reset pitch axis angle to sum of -90 degree and fine tune angle if gimbal downward,
        * sum of 90 degree and fine tune angle if upward. */
            GIMBAL_RESET_MODE_PITCH_DOWNWARD_UPWARD = 12,
        /*! Reset pitch axis of gimbal. Reset pitch axis angle to mapping angle */
            GIMBAL_RESET_MODE_PITCH_MAPPING = 13,
    };

    /**
     * @brief Input mode control gimbal
     * 
     */
    enum input_mode_t {
        INPUT_ANGLE = 1,
        INPUT_SPEED = 2
    };

    Gimbal_Protocol(Serial_Port *serial_port, const mavlink_system_t &system);
    virtual ~Gimbal_Protocol();

    /**
     * @brief Function init protocol
     * 
     * @param gimbal 
     */
    void initialize(const mavlink_system_t &gimbal);

    /**
     * @brief  This function set gimbal mode
     * @param: type see control_mode_t
     * @ret: result
     */
    virtual result_t set_gimbal_mode_sync(control_mode_t mode) = 0;

	/**
	 * @brief  This function reset gimbal with some mode
	 * @param: type see gimbal_reset_mode_t
	 * @ret: result 
	 */
    virtual result_t set_gimbal_reset_mode(gimbal_reset_mode_t reset_mode) = 0;

    /**
     * @brief Set the gimbal move sync
     * 
     * @param pitch control pitch value
     * @param roll control roll value
     * @param yaw control yaw value
     * @param mode see input_mode_t
     * @return result_t 
     */
    virtual result_t set_gimbal_move_sync(float pitch, float roll, float yaw, input_mode_t mode) = 0;

    /**
     * @brief Function send commad long to gimbal
     * 
     * @param command command ID
     * @param param param1 -> param7
     * @return result_t SUCCESS if send command successfully
     */
    result_t send_command_long(uint16_t command, const float param[7]);

    /**
     * @brief Function send commad long to gimbal and block to wait for ack response
     * 
     * @param command command ID
     * @param param param1 -> param7
     * @return result_t ack response from gimbal
     */
    result_t send_command_long_sync(uint16_t command, const float param[7]);

    /**
     * @brief callback for command ack
     * @details called from serial read message
     * @param command command ID
     * @param result MAV_RESULT
     */
    void command_ack_callback(const mavlink_message_t &message);

    // Helper
    static float to_deg(float rad) {
        static constexpr float RAD2DEG = 180.f / M_PI;
        return rad * RAD2DEG;
    }

    static float to_rad(float deg) {
        static constexpr float DEG2RAD = M_PI / 180.f;
        return deg * DEG2RAD;
    }

protected:

    Serial_Port *_serial_port;

    pthread_mutex_t _mutex;
    pthread_cond_t _condition;

    mavlink_system_t _system;
    mavlink_system_t _gimbal = { 0 };
    bool             _is_init = false;

    mavlink_command_ack_t _ack = { 0 };

    /**
     * @brief convert from mav result
     * 
     * @param res 
     * @return result_t 
     */
    result_t from_mav_result(MAV_RESULT res);
};

#endif // GIMBAL_PROTOCOL_H_
