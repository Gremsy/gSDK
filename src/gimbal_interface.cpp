/*******************************************************************************
 * @file    gimbal_interface.cpp
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

/* Includes ------------------------------------------------------------------*/

#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "gimbal_protocol_v1.h"
#include "gimbal_protocol_v2.h"

#include "gimbal_interface.h"

// Gimbal status 1
enum status1_t {
    STATUS1_ERROR_NONE          = 0x00,
    STATUS1_MODE_FOLLOW_LOCK    = 0x01,
    STATUS1_MISS_STEP           = 0x02,
    STATUS1_SENSOR_ERROR        = 0x04,
    STATUS1_BATT_LOW            = 0x08,
    STATUS1_MOTORS              = 0x10,         /// motors on = 1, motor off = 0 (fimware 1.3.4)*/
    STATUS1_INIT_MOTOR          = 0x20,
    STATUS1_AUTO_TUNER          = 0x40,         /// 0b0100 0000
    STATUS1_CANLINK             = 0x80,         /// 0b1000 0000 ket noi can link.
    STATUS1_SEARCH_HOME         = 0x100,        /// search home
    STATUS1_SET_HOME            = 0x200,        /// set home
    STATUS1_SENSOR_CALIB        = 0x400,        /// calib sensor gom accel va gyro
    STATUS1_STARTUP             = 0x800,
    STATUS1_REMOTE              = 0x1000,
    STATUS1_INVERTED            = 0x2000,
    STATUS1_MOTOR_PHASE_ERROR   = 0x4000,
    STATUS1_MOTOR_ANGLE_ERROR   = 0x8000,
};

// Gimbal status 2
enum status2_t {
    STATUS2_ERROR_NONE              = 0x00,
    STATUS2_IMU_ERROR               = 0x01,
    STATUS2_MOTOR_TILT_ERROR        = 0x02,
    STATUS2_MOTOR_ROLL_ERROR        = 0x04,
    STATUS2_MOTOR_PAN_ERROR         = 0x08,
    STATUS2_JOYSTICK_ERROR          = 0x10,
    STATUS2_INVERTED_ERROR          = 0x20,
    STATUS2_PAN_SEARCH_HOME_ERROR   = 0x40,

    STATUS2_ANGLE_TILT_ERROR        = 0x80,
    STATUS2_ANGLE_ROLL_ERROR        = 0x100,
    STATUS2_ANGLE_PAN_ERROR         = 0x200,

    STATUS2_MOVE_TILT_ERROR         = 0x400,
    STATUS2_MOVE_ROLL_ERROR         = 0x800,
    STATUS2_MOVE_PAN_ERROR          = 0x1000,
};

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t get_time_usec()
{
    static struct timeval _timestamp;
    gettimeofday(&_timestamp, NULL);
    return _timestamp.tv_sec * 1000000 + _timestamp.tv_usec;
}

uint64_t get_time_msec()
{
    static struct timeval _timestamp;
    gettimeofday(&_timestamp, NULL);
    return _timestamp.tv_sec * 1000 + _timestamp.tv_usec / 1000;
}

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
/**
 * @brief Construct a new Gimbal_Interface object
 *
 * @param serial_port serial object handle low level communication
 * @param sysid mavlink sysid of this system
 * @param compid mavlink compid of this system
 * @param proto MAVLink Gimbal Protocol version
 */
Gimbal_Interface::Gimbal_Interface(Serial_Port *serial_port,
                                   uint8_t sysid /*= 1 */,
                                   uint8_t compid /*= MAV_COMP_ID_ONBOARD_COMPUTER */,
                                   MAVLINK_PROTO proto /*= MAVLINK_GIMBAL_V2 */)
{
    // initialize attributes
    _system.sysid  = sysid;
    _system.compid = compid;
    _proto         = proto;
    _serial_port   = serial_port;

    if (_proto == MAVLINK_GIMBAL_V1) {
        _gimbal_proto = new Gimbal_Protocol_V1(serial_port, _system);

    } else {
        _gimbal_proto = new Gimbal_Protocol_V2(serial_port, _system);
    }
}

Gimbal_Interface::~Gimbal_Interface()
{
    delete _gimbal_proto;
}


// ------------------------------------------------------------------------------
//   Handle Messages
// ------------------------------------------------------------------------------
void Gimbal_Interface::messages_handler(const mavlink_message_t &message)
{
    // ----------------------------------------------------------------------
    //   HANDLE MESSAGE
    // ----------------------------------------------------------------------
    /* Only handle messages from gimbal */
    if (is_gimbal(message.compid)) {
        // Handle Message ID
        switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: {
                    pthread_mutex_lock(&_messages.mutex);
                    mavlink_msg_heartbeat_decode(&message, &_messages.heartbeat);
                    _messages.timestamps.heartbeat = get_time_usec();

                    if (has_detected == false) {
                        // Store message sysid and compid.
                        _gimbal.sysid  = message.sysid;
                        _gimbal.compid = message.compid;

                        if (_gimbal_proto != nullptr) {
                            _gimbal_proto->initialize(_gimbal);
                        }

                        has_detected = true;
                    }

                    pthread_mutex_unlock(&_messages.mutex);
                    break;
                }

            case MAVLINK_MSG_ID_SYS_STATUS: {
                    pthread_mutex_lock(&_messages.mutex);
                    mavlink_msg_sys_status_decode(&message, &_messages.sys_status);
                    _messages.timestamps.sys_status = get_time_usec();
                    pthread_mutex_unlock(&_messages.mutex);
                    break;
                }

            case MAVLINK_MSG_ID_MOUNT_STATUS: {
                    pthread_mutex_lock(&_messages.mutex);
                    mavlink_msg_mount_status_decode(&message, &_messages.mount_status);
                    _messages.timestamps.mount_status = get_time_usec();
                    pthread_mutex_unlock(&_messages.mutex);
                    break;
                }

            case MAVLINK_MSG_ID_MOUNT_ORIENTATION: {
                    pthread_mutex_lock(&_messages.mutex);
                    mavlink_msg_mount_orientation_decode(&message, &_messages.mount_orientation);
                    _messages.timestamps.mount_orientation = get_time_usec();

                    if (_gimbal_proto != nullptr) {
                        _gimbal_proto->update_attitude(_messages.mount_orientation.pitch,
                                                       _messages.mount_orientation.roll,
                                                       _messages.mount_orientation.yaw);
                    }

                    // printf("[%f][%f][%f]\n\r", _messages.mount_orientation.pitch, _messages.mount_orientation.roll, _messages.mount_orientation.yaw);

                    pthread_mutex_unlock(&_messages.mutex);
                    break;
                }

            case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: {
                    pthread_mutex_lock(&_messages.mutex);
                    mavlink_msg_gimbal_device_attitude_status_decode(&message, &_messages.atttitude_status);
                    _messages.timestamps.attitude_status = get_time_usec();

                    if (_gimbal_proto != nullptr) {
                        _gimbal_proto->update_attitude(_messages.atttitude_status.q);
                    }

                    pthread_mutex_unlock(&_messages.mutex);
                    break;
                }

            case MAVLINK_MSG_ID_RAW_IMU: {
                    pthread_mutex_lock(&_messages.mutex);
                    mavlink_msg_raw_imu_decode(&message, &_messages.raw_imu);
                    _messages.timestamps.raw_imu = get_time_usec();
                    pthread_mutex_unlock(&_messages.mutex);
                    break;
                }

            case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION: {
                    pthread_mutex_lock(&_messages.mutex);
                    mavlink_msg_gimbal_device_information_decode(&message, &_messages.gimbal_device_info);
                    _messages.timestamps.gimbal_device_info = get_time_usec();
                    printf("GET GIMBAL DEVICE: \n");
                    printf("Vendor name: %s\n", _messages.gimbal_device_info.vendor_name);
                    printf("Model name: %s\n", _messages.gimbal_device_info.model_name);
                    pthread_mutex_unlock(&_messages.mutex);
                    break;
                }

            case MAVLINK_MSG_ID_COMMAND_ACK: {
                    mavlink_command_ack_t packet = { 0 };
                    mavlink_msg_command_ack_decode(&message, &packet);
                    pthread_mutex_lock(&_messages.mutex);
                    _messages.timestamps.command_ack = get_time_usec();
                    pthread_mutex_unlock(&_messages.mutex);

                    if (_gimbal_proto != nullptr) {
                        _gimbal_proto->command_ack_callback(message);
                    }

                    break;
                }

            case MAVLINK_MSG_ID_PARAM_VALUE: {
                    mavlink_param_value_t packet = { 0 };
                    mavlink_msg_param_value_decode(&message, &packet);
                    pthread_mutex_lock(&_messages.mutex);
                    _messages.timestamps.param = get_time_usec();
                    pthread_mutex_unlock(&_messages.mutex);

                    for (uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
                        // Compare the index from gimbal with the param list
                        if (packet.param_index == _params_list[i].gmb_idx) {
                            _params_list[i].fetch_attempts = 0;
                            _params_list[i].seen           = true;

                            switch (_params_list[i].state) {
                                case PARAM_STATE_NONEXISTANT:
                                case PARAM_STATE_NOT_YET_READ:
                                case PARAM_STATE_FETCH_AGAIN:
                                    _params_list[i].value = packet.param_value;
                                    _params_list[i].state = PARAM_STATE_CONSISTENT;
                                    printf("GOT [%s] %d\n", get_param_name((param_index_t)i), _params_list[i].value);
                                    break;

                                case PARAM_STATE_CONSISTENT:
                                    _params_list[i].value = (int16_t)packet.param_value;
                                    break;

                                case PARAM_STATE_ATTEMPTING_TO_SET:
                                    if (packet.param_value == _params_list[i].value) {
                                        _params_list[i].state = PARAM_STATE_CONSISTENT;
                                    }

                                    break;
                            }

                            break;
                        }
                    }

                    break;
                }

            default: {
                    break;
                }
        } // end: switch msgid
    } // end: if read message
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int Gimbal_Interface::write_message(const mavlink_message_t &message)
{
    if (_serial_port == nullptr) {
        fprintf(stderr, "ERROR: serial port not exist\n");
        throw 1;
    }

    return _serial_port->write_message(message);
}

// ------------------------------------------------------------------------------
//   GIMBAL Parameters
// ------------------------------------------------------------------------------
void Gimbal_Interface::reset_params()
{
    _last_request_ms = 0;

    for (uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
        _params_list[i].value          = 0;
        _params_list[i].state          = PARAM_STATE_NOT_YET_READ;
        _params_list[i].fetch_attempts = 0;
        _params_list[i].seen           = 0;
    }
}

void Gimbal_Interface::fetch_params()
{
    for (uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
        /// Check the param has been read before
        if (_params_list[i].state != PARAM_STATE_NOT_YET_READ) {
            // Then set the state to allow read again
            _params_list[i].state = PARAM_STATE_FETCH_AGAIN;
        }
    }
}

bool Gimbal_Interface::params_initialized()
{
    for (uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (_params_list[i].state == PARAM_STATE_NOT_YET_READ) {
            return false;
        }
    }

    return true;
}


bool Gimbal_Interface::params_received_all()
{
    for (uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (_params_list[i].state == PARAM_STATE_NOT_YET_READ ||
                _params_list[i].state == PARAM_STATE_FETCH_AGAIN) {
            return false;
        }
    }

    return true;
}

Gimbal_Protocol::result_t Gimbal_Interface::get_param(param_index_t param, int16_t &value)
{
    if (_params_list[param].seen) {
        value = _params_list[param].value;
        return Gimbal_Protocol::SUCCESS;
    }

    return Gimbal_Protocol::Gimbal_Protocol::ERROR;
}

Gimbal_Protocol::result_t Gimbal_Interface::set_param(param_index_t param, int16_t value)
{
    if ((_params_list[param].state == PARAM_STATE_CONSISTENT) && (_params_list[param].value == value)) {
        return Gimbal_Protocol::SUCCESS;
    }

    if (_params_list[param].state == PARAM_STATE_NONEXISTANT) {
        return Gimbal_Protocol::ERROR;
    }

    _params_list[param].value = value;
    _params_list[param].state = PARAM_STATE_ATTEMPTING_TO_SET;
    // Prepare command for off-board mode
    mavlink_param_set_t param_set = { 0 };
    param_set.param_value		= value; 		/*<  Onboard parameter value*/
    param_set.target_system		= _gimbal.sysid; 	/*<  System ID*/
    param_set.target_component	= _gimbal.compid; 	/*<  Component ID*/
    strncpy(param_set.param_id, get_param_name(param), 16);
    param_set.param_type		= MAVLINK_TYPE_UINT16_T;
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = { 0 };
    mavlink_msg_param_set_encode(_system.sysid, _system.compid, &message, &param_set);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    // do the write
    if (write_message(message) <= 0) {
        fprintf(stderr, "WARNING: could not set param: %s\n", get_param_name(param));
    }

    _last_request_ms = get_time_msec();
    return Gimbal_Protocol::SUCCESS;
}

void Gimbal_Interface::param_update()
{
    uint64_t tnow_ms = get_time_msec();

    /// Retry initia param retrieval
    if (!params_received_all()) {
        for (uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
            if (!_params_list[i].seen && (tnow_ms - _last_request_ms > 10)) {
                if (request_param((param_index_t)i) == Gimbal_Protocol::SUCCESS) {
                    _last_request_ms = tnow_ms;
                    _params_list[i].fetch_attempts++;
                }
            }
        }
    }

    for (uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
        // retry param set
        if (_params_list[i].state == PARAM_STATE_ATTEMPTING_TO_SET && (tnow_ms - _last_request_ms > _RETRY_PERIOD)) {
            if (request_param((param_index_t)i) == Gimbal_Protocol::SUCCESS) {
                _params_list[i].state = PARAM_STATE_FETCH_AGAIN;
                _params_list[i].seen  = false;
                _last_set_ms = tnow_ms;
            }
        }

        // Check for nonexistent parameters
        if (!_params_list[i].seen && _params_list[i].fetch_attempts > _MAX_FETCH_TIME) {
            _params_list[i].state = PARAM_STATE_NONEXISTANT;
            printf("Gimbal parameter %s timed out\n", get_param_name((param_index_t)i));
        }
    }
}

// ------------------------------------------------------------------------------
//   Process when gimbal conntected
// ------------------------------------------------------------------------------
void Gimbal_Interface::param_process(void)
{
    if (!get_connection()) {
        _state = GIMBAL_STATE_NOT_PRESENT;
    }

    switch (_state) {
        case GIMBAL_STATE_NOT_PRESENT: {
                // gimbal was just connected or we just rebooted, transition to PRESENT_INITIALIZING
                reset_params();

                if (get_connection() && request_param_list() == Gimbal_Protocol::SUCCESS) {
                    _state = GIMBAL_STATE_PRESENT_IDLING;
                }
            }
            break;

        case GIMBAL_STATE_PRESENT_IDLING: {
                pthread_mutex_lock(&_messages.mutex);
                uint64_t param_timestamps = _messages.timestamps.param;
                pthread_mutex_unlock(&_messages.mutex);

                if (get_time_usec() - param_timestamps > 1000000) {
                    _state = GIMBAL_STATE_PRESENT_INITIALIZING;
                }
            }
            break;

        case GIMBAL_STATE_PRESENT_INITIALIZING: {
                param_update();

                // // parameters done initializing,
                if (params_initialized()) {
                    for (uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
                        printf("Check [%s] %d\n", _params_list[i].gmb_id, _params_list[i].value);
                    }

                    _state = GIMBAL_STATE_PRESENT_ALIGNING;
                }
            }
            break;

        case GIMBAL_STATE_PRESENT_ALIGNING: {
                param_update();
                pthread_mutex_lock(&_messages.mutex);
                uint16_t error_count2 = _messages.sys_status.errors_count2;
                pthread_mutex_unlock(&_messages.mutex);

                if (error_count2 == 0x00) {
                    _state = GIMBAL_STATE_PRESENT_RUNNING;

                } else {
                    printf("Gimbal Error with code: %d\n", error_count2);
                }
            }
            break;

        case GIMBAL_STATE_PRESENT_RUNNING: {
                param_update();
            }
            break;
    }
}

// ------------------------------------------------------------------------------
//   GIMBAL Command
// ------------------------------------------------------------------------------
/**
 * @brief  This function reboot the gimbal
 * @param: NONE
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_reboot(void)
{
    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    const float param[7] = {
        0,
        0,
        0,
        1,  // param 4
        0,
        0,
        0
    };
    return _gimbal_proto->send_command_long(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param);
}

/**
 * @brief  This function set gimbal to rc input mode and block to wait for gimbal response
 * @param: NONE
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_rc_input_sync(void)
{
    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    const float param[7] = {
        MAV_MOUNT_MODE_RC_TARGETING,
        0,
        0,
        0,
        0,
        0,
        0
    };
    return _gimbal_proto->send_command_long_sync(MAV_CMD_DO_MOUNT_CONFIGURE, param);
}

/**
 * @brief  This function set gimbal motor ON/OFF
 * @param: type see control_motor_t
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_motor(control_motor_t type)
{
    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    const float param[7] = {
        0,
        0,
        0,
        0,
        0,
        0,
        (float)type
    };
    return _gimbal_proto->send_command_long_sync(MAV_CMD_USER_1, param);
}

/**
 * @brief  This function get gimbal mode
 * @param: type see gimbal_mode_t
 * @ret: gimbal_mode_t
 */
Gimbal_Protocol::control_mode_t Gimbal_Interface::get_gimbal_mode(void)
{
    // Get gimbal status
    pthread_mutex_lock(&_messages.mutex);
    const uint16_t errors_count1 = _messages.sys_status.errors_count1;
    pthread_mutex_unlock(&_messages.mutex);

    /* Check gimbal's motor */
    if (errors_count1 & STATUS1_MOTORS) {
        _status.state = GIMBAL_STATE_ON;

        /* Check gimbal is follow mode*/
        if (errors_count1 & STATUS1_MODE_FOLLOW_LOCK) {
            _status.mode = Gimbal_Protocol::GIMBAL_FOLLOW_MODE;

        } else {
            _status.mode = Gimbal_Protocol::GIMBAL_LOCK_MODE;
        }
    }

    return (Gimbal_Protocol::control_mode_t)_status.mode;
}

/**
 * @brief  This function reset gimbal with some mode
 * @param: type see gimbal_reset_mode_t
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_reset_mode(Gimbal_Protocol::gimbal_reset_mode_t reset_mode)
{
    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    return _gimbal_proto->set_gimbal_reset_mode(reset_mode);
}

/**
 * @brief  This function set gimbal lock mode
 * @param: NONE
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_lock_mode_sync(void)
{
    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    return _gimbal_proto->set_gimbal_mode_sync(Gimbal_Protocol::control_mode_t::GIMBAL_LOCK_MODE);
}

/**
 * @brief  This function set gimbal follow mode
 * @param: NONE
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_follow_mode_sync(void)
{
    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    return _gimbal_proto->set_gimbal_mode_sync(Gimbal_Protocol::control_mode_t::GIMBAL_FOLLOW_MODE);
}

/**
 * @brief  This function rotate to target angle (deg)
 * @param: pitch control pitch value
 * @param: roll control roll value
 * @param: yaw control yaw value (-180; 180)
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_rotation_sync(float pitch, float roll, float yaw)
{
    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    return _gimbal_proto->set_gimbal_move_sync(pitch, roll, yaw, Gimbal_Protocol::INPUT_ANGLE);
}

/**
 * @brief  This function rotate target rate (deg/s)
 * @param: pitch control pitch value
 * @param: roll control roll value
 * @param: yaw control yaw value
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_rotation_rate_sync(float pitch, float roll, float yaw)
{
    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    return _gimbal_proto->set_gimbal_move_sync(pitch, roll, yaw, Gimbal_Protocol::INPUT_SPEED);
}

/**
 * @brief  This function use for move gimbal follow fight controller
 * @param: *pitch is pointer manager follow param of pitch axis of gimbal.
 * @param: *roll is pointer manager follow param of roll axis of gimbal.
 * @param: *yaw is pointer manager follow param of yaw axis of gimbal.
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_follow_fc_sync(gimbal_follow_param_t *pitch, gimbal_follow_param_t *roll, gimbal_follow_param_t *yaw)
{
    float delta[GIMBAL_TOTAL_AXIS]  = {0.f, 0.f, 0.f};
    float speed[GIMBAL_TOTAL_AXIS]  = {0.f, 0.f, 0.f};
    attitude<float> attitude;

    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    /// get gimbal attitude pitch, roll, yaw
    attitude = get_gimbal_attitude();

    /// calculator angle delta and speed
    if(pitch != NULL)
    {
        delta[GIMBAL_PITCH_AXIS] = pitch->setPoint - attitude.pitch;
        speed[GIMBAL_PITCH_AXIS] = pitch->ratio * delta[GIMBAL_PITCH_AXIS];
    }

    if(roll != NULL)
    {
        delta[GIMBAL_ROLL_AXIS] = roll->setPoint - attitude.roll;
        speed[GIMBAL_ROLL_AXIS] = roll->ratio * delta[GIMBAL_ROLL_AXIS];
    }

    if(yaw != NULL)
    {
        delta[GIMBAL_YAW_AXIS] = yaw->setPoint - attitude.yaw;
        speed[GIMBAL_YAW_AXIS] = yaw->ratio * delta[GIMBAL_YAW_AXIS];
    }
    
    /// limit speed
    if(speed[GIMBAL_PITCH_AXIS] > 180.f)
    {
        speed[GIMBAL_PITCH_AXIS] = 180.f;
    }
    else if(speed[GIMBAL_PITCH_AXIS] < -180.f)
    {
        speed[GIMBAL_PITCH_AXIS] = -180.f;
    }

    if(speed[GIMBAL_ROLL_AXIS] > 180.f)
    {
        speed[GIMBAL_ROLL_AXIS] = 180.f;
    }
    else if(speed[GIMBAL_ROLL_AXIS] < -180.f)
    {
        speed[GIMBAL_ROLL_AXIS] = -180.f;
    }

    if(speed[GIMBAL_YAW_AXIS] > 180.f)
    {
        speed[GIMBAL_YAW_AXIS] = 180.f;
    }
    else if(speed[GIMBAL_YAW_AXIS] < -180.f)
    {
        speed[GIMBAL_YAW_AXIS] = -180.f;
    }

    /// set move gimbal
    set_gimbal_rotation_rate_sync(speed[GIMBAL_PITCH_AXIS], speed[GIMBAL_ROLL_AXIS], speed[GIMBAL_YAW_AXIS]);

    return Gimbal_Protocol::SUCCESS;
}

/**
 * @brief  This function set motor controls setting
 * @param: tilt, roll, pan - stiffness and holdstrengtg, see user_manual (https://gremsy.com/gremsy-t3-manual/)
 * @param: def_gyro_filter - The coefficent for denoising the sensor filter
 * @param: def_output_filter - The coefficent for denoising the output filter
 * @param: def_gain - Defines how fast each axis will return to commanded position.
 * @ret: gimbal_motor_control_t contains setting related to tilt axis
 *
 *
 *	GYRO FILTER 	2
    *	OUTPUT FILTER 	3
    *
    *	HOLD STRENGTH 	TILT 	ROLL 	PAN
    *					40 		40 		40
    * 	GAIN 			120		120		120
    */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_motor_control(const gimbal_motor_control_t &tilt,
        const gimbal_motor_control_t &roll,
        const gimbal_motor_control_t &pan,
        uint8_t gyro_filter, uint8_t output_filter, uint8_t gain)
{
    return (set_param(GMB_PARAM_STIFFNESS_PITCH, (int16_t)tilt.stiffness) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_HOLDSTRENGTH_PITCH, (int16_t)tilt.holdstrength) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_STIFFNESS_ROLL, (int16_t)roll.stiffness) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_HOLDSTRENGTH_ROLL, (int16_t)roll.holdstrength) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_STIFFNESS_YAW, (int16_t)pan.stiffness) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_HOLDSTRENGTH_YAW, (int16_t)pan.holdstrength) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_OUTPUT_FILTER, (int16_t)output_filter) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_GYRO_FILTER, (int16_t)gyro_filter) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_GAIN, (int16_t)gain) == Gimbal_Protocol::SUCCESS) ? Gimbal_Protocol::SUCCESS : Gimbal_Protocol::ERROR;
}

/**
 * @brief  This function get motor controls setting
 * @param: tilt, roll, pan - stiffness and holdstrengtg, see user_manual (https://gremsy.com/gremsy-t3-manual/)
 * @param: def_gyro_filter - The coefficent for denoising the sensor filter
 * @param: def_output_filter - The coefficent for denoising the output filter
 * @param: def_gain - Defines how fast each axis will return to commanded position.
 * @ret: gimbal_motor_control_t contains setting related to tilt axis
 *
 *
 *	GYRO FILTER 	2
    *	OUTPUT FILTER 	3
    *
    *	HOLD STRENGTH 	TILT 	ROLL 	PAN
    *					40 		40 		40
    * 	GAIN 			120		120		120
    */
Gimbal_Protocol::result_t Gimbal_Interface::get_gimbal_motor_control(gimbal_motor_control_t &tilt,
        gimbal_motor_control_t &roll,
        gimbal_motor_control_t &pan,
        uint8_t &gyro_filter, uint8_t &output_filter, uint8_t &gain)
{
    int16_t value = 0;

    if (get_param(GMB_PARAM_STIFFNESS_PITCH, value) == Gimbal_Protocol::SUCCESS)
        tilt.stiffness = (uint8_t)value;

    if (get_param(GMB_PARAM_HOLDSTRENGTH_PITCH, value) == Gimbal_Protocol::SUCCESS)
        tilt.holdstrength = (uint8_t)value;

    if (get_param(GMB_PARAM_STIFFNESS_ROLL, value) == Gimbal_Protocol::SUCCESS)
        roll.stiffness = (uint8_t)value;

    if (get_param(GMB_PARAM_HOLDSTRENGTH_ROLL, value) == Gimbal_Protocol::SUCCESS)
        roll.holdstrength = (uint8_t)value;

    if (get_param(GMB_PARAM_STIFFNESS_YAW, value) == Gimbal_Protocol::SUCCESS)
        pan.stiffness = (uint8_t)value;

    if (get_param(GMB_PARAM_HOLDSTRENGTH_YAW, value) == Gimbal_Protocol::SUCCESS)
        pan.holdstrength = (uint8_t)value;

    if (get_param(GMB_PARAM_OUTPUT_FILTER, value) == Gimbal_Protocol::SUCCESS)
        output_filter = (uint8_t)value;

    if (get_param(GMB_PARAM_GYRO_FILTER, value) == Gimbal_Protocol::SUCCESS)
        gyro_filter	= (uint8_t)value;

    if (get_param(GMB_PARAM_GAIN, value) == Gimbal_Protocol::SUCCESS)
        gain = (uint8_t)value;

    return Gimbal_Protocol::SUCCESS;
}

/**
 * @brief  This function shall configure on the tilt axis
 * @param: config see  gimbal_config_axis_t structure
 * @note: The smooth starts with a low value of 50
 *			Slowly increase this setting until you feel an oscillation in the pan axis,
 *			then reduce the setting until the oscillation subsides.
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_config_tilt_axis(const gimbal_config_axis_t &config)
{
    int16_t get_dir = 0;

    if (get_param(GMB_PARAM_AXIS_DIR, get_dir) == Gimbal_Protocol::SUCCESS) {
        if (config.dir == DIR_CCW) {
            get_dir |= 0x01;

        } else {
            get_dir &= (~0x01);
        }

    } else {
        return Gimbal_Protocol::ERROR;
    }

    return (set_param(GMB_PARAM_SMOOTH_CONTROL_PITCH, (int16_t)config.smooth_control) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_SMOOTH_FOLLOW_PITCH, (int16_t)config.smooth_follow) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_WINDOW_FOLLOW_PITCH, (int16_t)config.window_follow) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_SPEED_FOLLOW_PITCH, (int16_t)config.speed_follow) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_SPEED_CONTROL_PITCH, (int16_t)config.speed_control) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_AXIS_DIR, get_dir) == Gimbal_Protocol::SUCCESS) ? Gimbal_Protocol::SUCCESS : Gimbal_Protocol::ERROR;
}

/**
 * @brief  This function get the config of tilt axis
 * @param: None
 * @ret: gimbal_config_axis_t contains setting related to tilt axis
 */
Gimbal_Interface::gimbal_config_axis_t Gimbal_Interface::get_gimbal_config_tilt_axis(void)
{
    gimbal_config_axis_t setting = { 0 };
    int16_t ret = 0;

    if (get_param(GMB_PARAM_SMOOTH_CONTROL_PITCH, ret) == Gimbal_Protocol::SUCCESS)
        setting.smooth_control = (uint8_t)ret;

    if (get_param(GMB_PARAM_SMOOTH_FOLLOW_PITCH, ret) == Gimbal_Protocol::SUCCESS)
        setting.smooth_follow = (uint8_t)ret;

    if (get_param(GMB_PARAM_WINDOW_FOLLOW_PITCH, ret) == Gimbal_Protocol::SUCCESS)
        setting.window_follow = (uint8_t)ret;

    if (get_param(GMB_PARAM_SPEED_FOLLOW_PITCH, ret) == Gimbal_Protocol::SUCCESS)
        setting.speed_follow = (uint8_t)ret;

    if (get_param(GMB_PARAM_SPEED_CONTROL_PITCH, ret) == Gimbal_Protocol::SUCCESS)
        setting.speed_control = (uint8_t)ret;

    if (get_param(GMB_PARAM_AXIS_DIR, ret) == Gimbal_Protocol::SUCCESS) {
        if (ret & 0x01) {
            setting.dir = DIR_CCW;

        } else if (!(ret & 0x01)) {
            setting.dir = DIR_CW;
        }
    }

    return setting;
}

/**
 * @brief  This function shall configure on the pan axis
 * @param: config see  gimbal_config_axis_t structure
 * @note: The smooth starts with a low value of 50
 *			Slowly increase this setting until you feel an oscillation in the pan axis,
 *			then reduce the setting until the oscillation subsides.
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_config_pan_axis(const gimbal_config_axis_t &config)
{
    int16_t get_dir = 0;

    if (get_param(GMB_PARAM_AXIS_DIR, get_dir) == Gimbal_Protocol::SUCCESS) {
        if (config.dir == DIR_CCW) {
            get_dir |= 0x02;

        } else {
            get_dir &= (~0x02);
        }

    } else {
        return Gimbal_Protocol::ERROR;
    }

    return (set_param(GMB_PARAM_SMOOTH_CONTROL_YAW, (int16_t)config.smooth_control) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_SMOOTH_FOLLOW_YAW, (int16_t)config.smooth_follow) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_WINDOW_FOLLOW_YAW, (int16_t)config.window_follow) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_SPEED_FOLLOW_YAW, (int16_t)config.speed_follow) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_SPEED_CONTROL_YAW, (int16_t)config.speed_control) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_AXIS_DIR, get_dir) == Gimbal_Protocol::SUCCESS) ? Gimbal_Protocol::SUCCESS : Gimbal_Protocol::ERROR;
}

/**
 * @brief  This function get the config of pan axis
 * @param: None
 * @ret: gimbal_config_axis_t contains setting related to pan axis
 */
Gimbal_Interface::gimbal_config_axis_t Gimbal_Interface::get_gimbal_config_pan_axis(void)
{
    gimbal_config_axis_t setting = { 0 };
    int16_t ret = 0;

    if (get_param(GMB_PARAM_SMOOTH_CONTROL_YAW, ret) == Gimbal_Protocol::SUCCESS)
        setting.smooth_control = (uint8_t)ret;

    if (get_param(GMB_PARAM_SMOOTH_FOLLOW_YAW, ret) == Gimbal_Protocol::SUCCESS)
        setting.smooth_follow = (uint8_t)ret;

    if (get_param(GMB_PARAM_WINDOW_FOLLOW_YAW, ret) == Gimbal_Protocol::SUCCESS)
        setting.window_follow = (uint8_t)ret;

    if (get_param(GMB_PARAM_SPEED_FOLLOW_YAW, ret) == Gimbal_Protocol::SUCCESS)
        setting.speed_follow = (uint8_t)ret;

    if (get_param(GMB_PARAM_SPEED_CONTROL_YAW, ret) == Gimbal_Protocol::SUCCESS)
        setting.speed_control = (uint8_t)ret;

    if (get_param(GMB_PARAM_AXIS_DIR, ret) == Gimbal_Protocol::SUCCESS) {
        if (ret & 0x02) {
            setting.dir = DIR_CCW;

        } else if (!(ret & 0x02)) {
            setting.dir = DIR_CW;
        }
    }

    return setting;
}

/**
 * @brief  This function shall configure on the roll axis
 * @param: config see  gimbal_config_axis_t structure
 * @note: The smooth starts with a low value of 50
 *			Slowly increase this setting until you feel an oscillation in the pan axis,
 *			then reduce the setting until the oscillation subsides.
 * @ret: result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_config_roll_axis(const gimbal_config_axis_t &config)
{
    int16_t get_dir = 0;

    if (get_param(GMB_PARAM_AXIS_DIR, get_dir) == Gimbal_Protocol::SUCCESS) {
        if (config.dir == DIR_CCW) {
            get_dir |= 0x04;

        } else {
            get_dir &= (~0x04);
        }

    } else {
        return Gimbal_Protocol::ERROR;
    }

    return (set_param(GMB_PARAM_SMOOTH_CONTROL_ROLL, (int16_t)config.smooth_control) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_SPEED_CONTROL_ROLL, (int16_t)config.smooth_follow) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_AXIS_DIR, get_dir) == Gimbal_Protocol::SUCCESS) ? Gimbal_Protocol::SUCCESS : Gimbal_Protocol::ERROR;
}

/**
 * @brief  This function get the config of roll axis
 * @param: None
 * @ret: gimbal_config_axis_t contains setting related to roll axis
 */
Gimbal_Interface::gimbal_config_axis_t Gimbal_Interface::get_gimbal_config_roll_axis(void)
{
    gimbal_config_axis_t setting = { 0 };
    int16_t ret = 0;

    if (get_param(GMB_PARAM_SMOOTH_CONTROL_ROLL, ret) == Gimbal_Protocol::SUCCESS)
        setting.smooth_control = (uint8_t)ret;

    if (get_param(GMB_PARAM_SPEED_CONTROL_ROLL, ret) == Gimbal_Protocol::SUCCESS)
        setting.smooth_follow = (uint8_t)ret;

    if (get_param(GMB_PARAM_AXIS_DIR, ret) == Gimbal_Protocol::SUCCESS) {
        if (ret & 0x04) {
            setting.dir = DIR_CCW;

        } else if (!(ret & 0x04)) {
            setting.dir = DIR_CW;
        }
    }

    return setting;
}

/**
 * @brief  This function get gimbal status
 * @param: None
 * @ret: Gimbal status
 */
Gimbal_Interface::gimbal_status_t Gimbal_Interface::get_gimbal_status(void)
{
    pthread_mutex_lock(&_messages.mutex);

    /* Check gimbal status has changed*/
    if (_messages.timestamps.sys_status) {
        /* Reset timestamps */
        _messages.timestamps.sys_status = 0;
        // Get gimbal status
        uint16_t errors_count1 = _messages.sys_status.errors_count1;
        uint16_t errors_count2 = _messages.sys_status.errors_count2;

        /* Check gimbal's motor */
        if (errors_count1 & STATUS1_MOTORS) {
            _status.state = GIMBAL_STATE_ON;

            /* Check gimbal is follow mode*/
            if (errors_count1 & STATUS1_MODE_FOLLOW_LOCK) {
                _status.mode = GIMBAL_STATE_FOLLOW_MODE;

            } else {
                _status.mode = GIMBAL_STATE_LOCK_MODE;
            }

        } else if (not (errors_count1 & STATUS1_MOTORS)) {
            _status.state = GIMBAL_STATE_OFF;
            _status.mode  = GIMBAL_STATE_OFF;
        }

        /* Check gimbal is initializing*/
        else if (errors_count1 & STATUS1_INIT_MOTOR) {
            _status.state = GIMBAL_STATE_INIT;

        } else if ((errors_count1 & STATUS1_SENSOR_ERROR) ||
                   (errors_count1 & STATUS1_MOTOR_PHASE_ERROR) ||
                   (errors_count1 & STATUS1_MOTOR_ANGLE_ERROR)) {
            /* Check gimbal is error state*/
            _status.state = GIMBAL_STATE_ERROR;
        }

        /* Check gimbal's sensor status */
        if (errors_count2 & 0x01) {
            _status.sensor |= SENSOR_IMU_ERROR;
        }

        if (errors_count2 & 0x02) {
            _status.sensor |= SENSOR_EN_TILT;
        }

        if (errors_count2 & 0x04) {
            _status.sensor |= SENSOR_EN_ROLL;
        }

        if (errors_count2 & 0x08) {
            _status.sensor |= SENSOR_EN_PAN;

        } else {
            _status.sensor = SENSOR_OK;
        }

    } else {
        /* Timeout status msg, reset */
        _status.state = GIMBAL_STATE_OFF;
        _status.mode  = GIMBAL_STATE_OFF;
    }

    pthread_mutex_unlock(&_messages.mutex);
    return _status;
}

/**
 * @brief  This function get gimbal imu raw values
 * @param: None
 * @ret: Gimbal raw imu
 */
Gimbal_Interface::imu_t Gimbal_Interface::get_gimbal_raw_imu(void)
{
    pthread_mutex_lock(&_messages.mutex);

    /* Check gimbal imu value has changed*/
    if (_messages.timestamps.raw_imu) {
        /* Reset timestamps */
        _messages.timestamps.raw_imu = 0;
        const mavlink_raw_imu_t &raw = _messages.raw_imu;
        return imu_t(vector3<int16_t>(raw.xacc, raw.yacc, raw.zacc), vector3<int16_t>(raw.xgyro, raw.ygyro, raw.zgyro));
    }

    pthread_mutex_unlock(&_messages.mutex);
    return imu_t();
}

/**
 * @brief  This function get gimbal attitude (deg)
 * @param: None
 * @ret: Gimbal attitude
 */
attitude<float> Gimbal_Interface::get_gimbal_attitude(void)
{
    uint64_t timestamps = 0;

    if (_proto == MAVLINK_GIMBAL_V1) {
        pthread_mutex_lock(&_messages.mutex);
        timestamps = _messages.timestamps.mount_orientation;
        pthread_mutex_unlock(&_messages.mutex);

        /* Check gimbal status has changed*/
        if (timestamps) {
            pthread_mutex_lock(&_messages.mutex);
            /* Reset timestamps */
            _messages.timestamps.mount_orientation = 0;
            const mavlink_mount_orientation_t &orient = _messages.mount_orientation;
            pthread_mutex_unlock(&_messages.mutex);
            return attitude<float>(orient.roll, orient.pitch, orient.yaw);
        }

    } else {
        pthread_mutex_lock(&_messages.mutex);
        timestamps = _messages.timestamps.attitude_status;
        pthread_mutex_unlock(&_messages.mutex);

        /* Check gimbal status has changed*/
        if (_messages.timestamps.attitude_status) {
            pthread_mutex_lock(&_messages.mutex);
            /* Reset timestamps */
            _messages.timestamps.attitude_status = 0;
            const mavlink_gimbal_device_attitude_status_t &status = _messages.atttitude_status;
            attitude<float> attitude;
            mavlink_quaternion_to_euler(status.q, &attitude.roll, &attitude.pitch, &attitude.yaw);
            pthread_mutex_unlock(&_messages.mutex);
            return attitude.to_deg();
        }
    }

    return attitude<float>();
}

/**
 * @brief  This function get gimbal encoder depends on encoder type send
 * @param: None
 * @ret: Gimbal encoder
 */
attitude<int16_t> Gimbal_Interface::get_gimbal_encoder(void)
{
    pthread_mutex_lock(&_messages.mutex);
    uint64_t timestamps = _messages.timestamps.mount_status;
    pthread_mutex_unlock(&_messages.mutex);

    /* Check gimbal encoder value has changed*/
    if (timestamps) {
        pthread_mutex_lock(&_messages.mutex);
        /* Reset timestamps */
        _messages.timestamps.mount_status = 0;
        const mavlink_mount_status_t &mount = _messages.mount_status;
        attitude<int16_t> encoder(mount.pointing_b, mount.pointing_a, mount.pointing_c);
        pthread_mutex_unlock(&_messages.mutex);
        return encoder;
    }

    return attitude<int16_t>();
}

/**
 * @brief  This function get gimbal timestamps
 * @param: None
 * @ret: Gimbal status
 */
Gimbal_Interface::timestamps_t Gimbal_Interface::get_gimbal_timestamps(void)
{
    pthread_mutex_lock(&_messages.mutex);
    timestamps_t timestamp = _messages.timestamps;
    pthread_mutex_unlock(&_messages.mutex);
    return timestamp;
}

/**
 * @brief  This function set the configuration the message mavink with rate
 * @param: encoder_rate - the time rate of the encoder values. Default 10Hz
 * @param: mnt_orient_rate - the time rate of the mount orientation. Default 10Hz
 * @param: attitude_status_rate - the time rate of the attitude status. Default 10Hz
 * @param: raw_imu_rate - the time rate of the raw_imu value. Default 10Hz
 * @NOTE The range [0 - 200Hz]. 0 will disable that message. Only set 1 msg rate higher than 50Hz
 * @ret: None
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_msg_encoder_rate(uint8_t rate)
{
    return set_msg_rate(MAVLINK_MSG_ID_MOUNT_STATUS, rate);
}

Gimbal_Protocol::result_t Gimbal_Interface::set_msg_mnt_orient_rate(uint8_t rate)
{
    return set_msg_rate(MAVLINK_MSG_ID_MOUNT_ORIENTATION, rate);
}

Gimbal_Protocol::result_t Gimbal_Interface::set_msg_attitude_status_rate(uint8_t rate)
{
    return set_msg_rate(MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, rate);
}

Gimbal_Protocol::result_t Gimbal_Interface::set_msg_raw_imu_rate(uint8_t rate)
{
    return set_msg_rate(MAVLINK_MSG_ID_RAW_IMU, rate);
}

/**
 * @brief Set the gimbal encoder type send
 * @param: true send raw encoder values
 *         false send encoder angle
 * @return result
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_encoder_type_send(bool type)
{
    return set_param(GMB_PARAM_ENCODER_TYPE, (int16_t)type);
}

/**
 * @brief reuqest gimbal device info message
 *
 * @return Gimbal_Protocol::result_t
 */
Gimbal_Protocol::result_t Gimbal_Interface::request_gimbal_device_info(void)
{
    return request_msg(MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION);
}

/**
 * @brief Get the gimbal encoder type send
 *
 * @return true send raw encoder values
 * @return false send encoder angle
 */
bool Gimbal_Interface::get_gimbal_encoder_type_send(void)
{
    int16_t ret = 0;
    get_param(GMB_PARAM_ENCODER_TYPE, ret);
    return (bool)ret;
}

/**
 * @brief  This function set the enable or disable the reduce drift of the gimbal by using attitude of the aircarf
 * @details Only enable yaw drift if there is an autopilot mount on top the gimbal and send autopilot's attitude
 * to gimbal at rate 50 - 100Hz
 * @param: flag - enable/disable the recude drift of the gimbal by combining attitude from the aircraft
 * @ret: None
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_gimbal_combine_attitude(bool flag)
{
    int16_t param = 0;

    if (get_param(GMB_PARAM_AXIS_DIR, param) == Gimbal_Protocol::SUCCESS) {
        if (flag) {
            param |= 0x10;

        } else {
            param &= (~0x10);
        }

        return set_param(GMB_PARAM_AXIS_DIR, param);
    }

    return Gimbal_Protocol::ERROR;
}

/**
 * @brief Set limit angle for pitch.
 * @details Please refer to Gremsy site <gremsy.com> for
 * details about default limit angle of Gimbal.
 * @param limitAngle: limit angle.
 * @return None
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_limit_angle_pitch(const limit_angle_t &limit_angle)
{
    return (set_param(GMB_PARAM_MIN_LIMIT_ANGLE_PITCH, (int16_t)limit_angle.angle_min) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_MAX_LIMIT_ANGLE_PITCH, (int16_t)limit_angle.angle_max) == Gimbal_Protocol::SUCCESS) ? Gimbal_Protocol::SUCCESS : Gimbal_Protocol::ERROR;
}

/**
 * @brief Get limit angle for pitch.
 * @details Please refer to Gremsy site <gremsy.com> for
 * details about default limit angle of Gimbal.
 * @param None
 * @return limit angle
 */
Gimbal_Interface::limit_angle_t Gimbal_Interface::get_limit_angle_pitch(void)
{
    limit_angle_t limit_angle = { 0 };
    int16_t ret = 0;

    if (get_param(GMB_PARAM_MIN_LIMIT_ANGLE_PITCH, ret) == Gimbal_Protocol::SUCCESS)
        limit_angle.angle_min = ret;

    if (get_param(GMB_PARAM_MAX_LIMIT_ANGLE_PITCH, ret) == Gimbal_Protocol::SUCCESS)
        limit_angle.angle_max = ret;

    return limit_angle;
}

/**
 * @brief Set limit angle for yaw.
 * @details Please refer to Gremsy site <gremsy.com> for
 * details about default limit angle of Gimbal.
 * @param limitAngle: limit angle.
 * @return None
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_limit_angle_yaw(const limit_angle_t &limit_angle)
{
    return (set_param(GMB_PARAM_MIN_LIMIT_ANGLE_YAW, (int16_t)limit_angle.angle_min) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_MAX_LIMIT_ANGLE_YAW, (int16_t)limit_angle.angle_max) == Gimbal_Protocol::SUCCESS) ? Gimbal_Protocol::SUCCESS : Gimbal_Protocol::ERROR;
}

/**
 * @brief Get limit angle for yaw.
 * @details Please refer to Gremsy site <gremsy.com> for
 * details about default limit angle of Gimbal.
 * @param None
 * @return limit angle
 */
Gimbal_Interface::limit_angle_t Gimbal_Interface::get_limit_angle_yaw(void)
{
    limit_angle_t limit_angle = { 0 };
    int16_t ret = 0;

    if (get_param(GMB_PARAM_MIN_LIMIT_ANGLE_YAW, ret) == Gimbal_Protocol::SUCCESS)
        limit_angle.angle_min = ret;

    if (get_param(GMB_PARAM_MAX_LIMIT_ANGLE_YAW, ret) == Gimbal_Protocol::SUCCESS)
        limit_angle.angle_max = ret;

    return limit_angle;
}

/**
 * @brief Set limit angle for roll.
 * @details Please refer to Gremsy site <gremsy.com> for
 * details about default limit angle of Gimbal.
 * @param limitAngle: limit angle.
 * @return None
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_limit_angle_roll(const limit_angle_t &limit_angle)
{
    return (set_param(GMB_PARAM_MIN_LIMIT_ANGLE_ROLL, (int16_t)limit_angle.angle_min) == Gimbal_Protocol::SUCCESS &&
            set_param(GMB_PARAM_MAX_LIMIT_ANGLE_ROLL, (int16_t)limit_angle.angle_max) == Gimbal_Protocol::SUCCESS) ? Gimbal_Protocol::SUCCESS : Gimbal_Protocol::ERROR;
}

/**
 * @brief Get limit angle for roll.
 * @details Please refer to Gremsy site <gremsy.com> for
 * details about default limit angle of Gimbal.
 * @param None
 * @return limit angle
 */
Gimbal_Interface::limit_angle_t Gimbal_Interface::get_limit_angle_roll(void)
{
    limit_angle_t limit_angle = { 0 };
    int16_t ret = 0;

    if (get_param(GMB_PARAM_MIN_LIMIT_ANGLE_ROLL, ret) == Gimbal_Protocol::SUCCESS)
        limit_angle.angle_min = ret;

    if (get_param(GMB_PARAM_MAX_LIMIT_ANGLE_ROLL, ret) == Gimbal_Protocol::SUCCESS)
        limit_angle.angle_max = ret;

    return limit_angle;
}

/**
 * @brief Set the external RC type
 * @param type rc type
 *
 * @return Gimbal_Protocol::result_t
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_rc_type(rc_type_t type)
{
    return set_param(GMB_PARAM_RC_TYPE, (int16_t) type);
}

/**
 * @brief Function check if compid is gimbal
 *
 * @param compid
 * @return true compid is gimbal
 * @return false
 */
bool Gimbal_Interface::is_gimbal(uint8_t compid)
{
    return (compid >= MAV_COMP_ID_GIMBAL && compid <= MAV_COMP_ID_GIMBAL6);
}

/**
 * @brief Set the msg rate
 *
 * @param msgid
 * @param rate
 * @return result_t
 */
Gimbal_Protocol::result_t Gimbal_Interface::set_msg_rate(uint32_t msgid, uint8_t rate)
{
    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    const float param[7] = {
        (float)msgid,
        1000000.f / rate,
        0,
        0,
        0,
        0,
        0
    };
    return _gimbal_proto->send_command_long_sync(MAV_CMD_SET_MESSAGE_INTERVAL, param);
}

/**
 * @brief Request msg from gimbal
 *
 * @param msgid
 * @return result_t
 */
Gimbal_Protocol::result_t Gimbal_Interface::request_msg(uint32_t msgid)
{
    if (_gimbal_proto == nullptr) {
        return Gimbal_Protocol::ERROR;
    }

    const float param[7] = {
        (float)msgid,
        0,
        0,
        0,
        0,
        0,
        0
    };
    return _gimbal_proto->send_command_long_sync(MAV_CMD_REQUEST_MESSAGE, param);
}

Gimbal_Protocol::result_t Gimbal_Interface::request_param(param_index_t param)
{
    mavlink_param_request_read_t request = { 0 };
    request.target_system    = _gimbal.sysid;
    request.target_component = _gimbal.compid;
    request.param_index      = _params_list[param].gmb_idx;
    strncpy(request.param_id, _params_list[param].gmb_id, 16);
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = { 0 };
    mavlink_msg_param_request_read_encode(_system.sysid, _system.compid, &message, &request);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    if (write_message(message) <= 0) {
        fprintf(stderr, "WARNING: could not send PARAM_REQUEST_READ\n");
        return Gimbal_Protocol::ERROR;
    }

    return Gimbal_Protocol::SUCCESS;
}

Gimbal_Protocol::result_t Gimbal_Interface::request_param_list(void)
{
    mavlink_param_request_list_t request = { 0 };
    request.target_system    = _gimbal.sysid;
    request.target_component = _gimbal.compid;
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = { 0 };
    mavlink_msg_param_request_list_encode(_system.sysid, _system.compid, &message, &request);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    if (write_message(message) <= 0) {
        fprintf(stderr, "WARNING: could not send PARAM_REQUEST_LIST\n");
        return Gimbal_Protocol::ERROR;
    }

    return Gimbal_Protocol::SUCCESS;
}

// ------------------------------------------------------------------------------
//   Write heartbeat Message
// ------------------------------------------------------------------------
void Gimbal_Interface::write_heartbeat(void)
{
    mavlink_heartbeat_t heartbeat = { 0 };
    heartbeat.type 			= MAV_TYPE_ONBOARD_CONTROLLER;
    heartbeat.autopilot 	= MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode 	= 0;
    heartbeat.custom_mode 	= 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = { 0 };
    mavlink_msg_heartbeat_encode(_system.sysid, _system.compid, &message, &heartbeat);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    if (write_message(message) <= 0)
        fprintf(stderr, "WARNING: could not send HEARTBEAT\n");
}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void Gimbal_Interface::start()
{
    int result = 0;

    // --------------------------------------------------------------------------
    //   CHECK SERIAL PORT
    // --------------------------------------------------------------------------
    if (_serial_port != nullptr) {
        if (_serial_port->status != SERIAL_PORT_OPEN) { // SERIAL_PORT_OPEN
            fprintf(stderr, "ERROR: serial port not open\n");
            throw 1;
        }

    } else {
        fprintf(stderr, "ERROR: serial port not exist\n");
        throw 1;
    }

    // --------------------------------------------------------------------------
    //   WRITE THREAD
    // --------------------------------------------------------------------------
    printf("START WRITE THREAD \n");
    result = pthread_create(&write_tid, NULL, &start_gimbal_interface_write_thread, this);

    if (result) throw result;

    // wait for it to be started
    while (writing_status == THREAD_NOT_INIT)
        usleep(100000); // 10Hz

    // now we're streaming HEARTBEAT
    printf("\n");
    // --------------------------------------------------------------------------
    //   READ THREAD
    // --------------------------------------------------------------------------
    printf("START READ THREAD \n");
    result = pthread_create(&read_tid, NULL, &start_gimbal_interface_read_thread, this);

    if (result) throw result;

    // now we're reading messages
    printf("\n");

    // --------------------------------------------------------------------------
    //   CHECK FOR MESSAGES
    // --------------------------------------------------------------------------
    do {
        if (time_to_exit) {
            printf("WAIT FOR MESSAGES FROM GIMBAL SysID %d\n", _system.sysid);
            return;
        }

        usleep(500000); // Check at 2Hz
    } while (!get_connection());

    printf("Found GIMBAL [SysID][CompID]: [%d][%d]\n", _gimbal.sysid, _gimbal.compid);
    // We know the gimbal is sending messages
    printf("\n");
    // Done!
}

// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void Gimbal_Interface::stop()
{
    // --------------------------------------------------------------------------
    //   CLOSE THREADS
    // --------------------------------------------------------------------------
    printf("CLOSE THREADS\n");
    // signal exit
    time_to_exit = true;
    // wait for exit
    pthread_join(read_tid, NULL);
    pthread_join(write_tid, NULL);
    // now the read and write threads are closed
    printf("\n");
    // still need to close the _serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void Gimbal_Interface::start_read_thread()
{
    if (reading_status != THREAD_NOT_INIT) {
        fprintf(stderr, "Read thread already running\n");

    } else {
        read_thread();
    }
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void Gimbal_Interface::start_write_thread(void)
{
    if (writing_status != THREAD_NOT_INIT) {
        fprintf(stderr, "Write thread already running\n");

    } else {
        write_thread();
    }
}

// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void Gimbal_Interface::handle_quit( int sig )
{
    // Send command disable
    // disable_offboard_control();
    try {
        stop();

    } catch (int error) {
        fprintf(stderr, "Warning, could not stop gimbal interface\n");
    }
}

bool Gimbal_Interface::get_flag_exit(void)
{
    return time_to_exit;
}

bool Gimbal_Interface::get_connection(void)
{
    pthread_mutex_lock(&_messages.mutex);
    uint64_t timeout = get_time_usec() - _messages.timestamps.heartbeat;
    pthread_mutex_unlock(&_messages.mutex);

    // Check heartbeat from gimbal
    if (!has_detected && timeout > _TIME_LOST_CONNECT) {
        printf(" Lost Connection!\n");
        // gimbal went away
        return false;
    }

    return true;
}

bool Gimbal_Interface::present()
{
    pthread_mutex_lock(&_messages.mutex);
    uint64_t timeout = get_time_usec() - _messages.timestamps.heartbeat;
    pthread_mutex_unlock(&_messages.mutex);

    // Check time out
    if (_state != GIMBAL_STATE_NOT_PRESENT && timeout > _TIME_LOST_CONNECT) {
        printf(" Not Present!\n");
        // gimbal went away
        _state = GIMBAL_STATE_NOT_PRESENT;
        return false;
    }

    return _state == GIMBAL_STATE_PRESENT_RUNNING;
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void Gimbal_Interface::read_thread(void)
{
    mavlink_message_t message     = { 0 };
    mavlink_status_t mav_status;

    if (_serial_port == nullptr) {
        fprintf(stderr, "ERROR: serial port not exist\n");
        throw 1;
    }

    while (!time_to_exit) {
        if (writing_status != THREAD_RUNNING) {
            reading_status = THREAD_RUNNING;
            // ----------------------------------------------------------------------
            //   READ MESSAGE
            // ----------------------------------------------------------------------
            char buf[256] = { 0 };
            int result = _serial_port->read_message(buf, 256);

            // --------------------------------------------------------------------------
            //   PARSE MESSAGE
            // --------------------------------------------------------------------------
            if (result > 0) {
                for (int i = 0; i < result; ++i) {
                    // the parsing
                    if (mavlink_parse_char(MAVLINK_COMM_1, (uint8_t)buf[i], &message, &mav_status)) {
                        messages_handler(message);
                    }
                }
            }

            // Couldn't read from port
            else {
                fprintf(stderr, "ERROR: Could not read port\n");
            }

            reading_status = THREAD_IDLING;
        }

        usleep(100);   // sleep 1000us
    } // end: while not received all
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void Gimbal_Interface::write_thread(void)
{
    uint64_t tnow_ms = 0;
    uint64_t time_send_param_ms = 0, time_send_heartbeat_ms = 0;

    // Blocking wait for new data
    while (!time_to_exit) {
        writing_status = THREAD_RUNNING;
        tnow_ms = get_time_msec();

        if (tnow_ms - time_send_heartbeat_ms >= 1000) {
            time_send_heartbeat_ms = get_time_msec();
            // write a message and signal writing
            write_heartbeat();

        } else if (tnow_ms - time_send_param_ms >= 500) {
            time_send_param_ms = get_time_msec();
            // Process check param
            param_process();
        }

        writing_status = THREAD_IDLING;
        usleep(1000);
    }
}

// End Gimbal_Interface

// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------
void *start_gimbal_interface_read_thread(void *args)
{
    // takes an gimbal object argument
    Gimbal_Interface *gimbal_interface = (Gimbal_Interface *)args;
    // run the object's read thread
    gimbal_interface->start_read_thread();
    // done!
    return NULL;
}

void *start_gimbal_interface_write_thread(void *args)
{
    // takes an gimbal object argument
    Gimbal_Interface *gimbal_interface = (Gimbal_Interface *)args;
    // run the object's read thread
    gimbal_interface->start_write_thread();
    // done!
    return NULL;
}

/************************ (C) COPYRIGHT Gremsy *****END OF FILE****************/
