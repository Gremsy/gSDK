/*******************************************************************************
 * @file    gsdk_types.h
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

#ifndef GSDK_TYPES_H_
#define GSDK_TYPES_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <cstdlib>
#include <cstdint>
#include <cmath>

namespace GSDK
{
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
     * @brief Control direction
     */
    enum control_direction_t {
        DIR_CW  = 0x00,
        DIR_CCW = 0x01
    };

    /**
     * @brief Control motor mode ON/OFF
     */
    enum control_motor_t {
        TURN_OFF = 0x00,
        TURN_ON  = 0x01
    };

    /**
     * @brief Gimbal Operation status
     */
    enum operation_state_t {
        GIMBAL_STATE_OFF         = 0x00,   /*< Gimbal is off*/
        GIMBAL_STATE_INIT        = 0x01,   /*< Gimbal is initializing*/
        GIMBAL_STATE_ON          = 0x02,   /*< Gimbal is on */
        GIMBAL_STATE_LOCK_MODE   = 0x04,
        GIMBAL_STATE_FOLLOW_MODE = 0x08,
        GIMBAL_STATE_SEARCH_HOME = 0x10,
        GIMBAL_STATE_SET_HOME    = 0x20,
        GIMBAL_STATE_ERROR       = 0x40
    };

    /**
     * @brief MAVLink Gimbal Protocol
     *
     */
    enum MAVLINK_PROTO {
        MAVLINK_GIMBAL_V1,
        MAVLINK_GIMBAL_V2
    };

    /**
     * @brief Gimbal remote controller type
     *
     */
    enum rc_type_t {
        RC_TYPE_SBUS_FASST = 1,
        RC_TYPE_SBUS_SFHSS = 13,

        RC_TYPE_JOY        = 2,

        RC_TYPE_JR_11BIT   = 4,
        RC_TYPE_JR_10BIT   = 5,

        RC_TYPE_PPM        = 6,

        RC_TYPE_SYNC       = 7,
        RC_TYPE_SYNC2      = 12,

        RC_TYPE_LB2_SINGLE = 10,
        RC_TYPE_LB2_NORMAL = 11,

        RC_TYPE_HERELINK   = 14,
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

    template<typename T>
    struct attitude {
        T roll  = 0;
        T pitch = 0;
        T yaw   = 0;

        static constexpr float DEG2RAD = M_PI / 180.f;
        static constexpr float RAD2DEG = 180.f / M_PI;

        attitude() = default;
        attitude(T val) : roll(val), pitch(val), yaw(val) {};
        attitude(T r, T p, T y) : roll(r), pitch(p), yaw(y) {};

        attitude &to_rad(void)
        {
            roll  *= DEG2RAD;
            pitch *= DEG2RAD;
            yaw   *= DEG2RAD;
            return *this;
        }

        attitude &to_deg(void)
        {
            roll  *= RAD2DEG;
            pitch *= RAD2DEG;
            yaw   *= RAD2DEG;
            return *this;
        }
    };

    template<typename T>
    struct vector3 {
        T x = 0;
        T y = 0;
        T z = 0;

        vector3() = default;
        vector3(T val) : x(val), y(val), z(val) {};
        vector3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {};
    };

    /**
     * @brief Firmware version
     *
     */
    struct fw_version_t {
        uint8_t x;
        uint8_t y;
        uint8_t z;
        const char *type;
    };

    /**
     * @brief Gimbal Status
     */
    struct gimbal_status_t {
        uint16_t load;			  /*< [ms] Maximum usage the mainloop time. Values: [0-1000] - should always be below 1000*/
        uint16_t voltage_battery; /*< [V] Battery voltage*/
        uint8_t sensor;			  /*< Specific sensor occur error (encorder, imu) refer sensor_state_t*/
        uint8_t state;			  /* System state of gimbal. Refer interface_state_t*/
        uint8_t mode;
    };

    /**
     * @brief gimbal_config_axis_t
     * This structure will contain the gimbal configuration related to speed, smooth, direction
     */
    struct gimbal_config_axis_t {
        int8_t dir;
        uint8_t speed_control;
        uint8_t smooth_control;

        uint8_t speed_follow;
        uint8_t smooth_follow;
        uint8_t window_follow;
    };

    /**
     * @brief gimbal_motor_control_t
     * stifness: Stiffness setting has a significant impact on the performance of the Gimbal.
     *			This setting adjusts the degrees to which the gimbal tries to correct
    *			for unwanted camera movement and hold the camera stable.
    * 			The higher you can run the setting without vibration or oscillation, the better.
    * Holdstrength: Power level required for the corresponding axis.
    *				This option is only recommended for advanced users. Set 40 as defaults
    */
    struct gimbal_motor_control_t {
        uint8_t stiffness;
        uint8_t holdstrength;
    };

    /**
     * @brief Limit angle data structure
     */
    struct limit_angle_t {
        int16_t angle_min;
        int16_t angle_max;
    };

    /**
     * @brief imu data type
     *
     */
    struct imu_t {
        vector3<int16_t> accel;
        vector3<int16_t> gyro;

        imu_t() : accel(0), gyro(0) {};
        imu_t(const vector3<int16_t> &_a, const vector3<int16_t> &_g) :
            accel(_a), gyro(_g) {};
    };

} // GSDK

#endif  // GSDK_TYPES_H_

