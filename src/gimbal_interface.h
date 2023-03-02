/*******************************************************************************
 * @file    gimbal_interface.h
 * @author  The GremsyCo
 * @version V2.3.0
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

#ifndef GIMBAL_INTERFACE_H_
#define GIMBAL_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "gimbal_protocol.h"

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------
// Helper functions
uint64_t get_time_usec();
uint64_t get_time_msec();
void *start_gimbal_interface_read_thread(void *args);
void *start_gimbal_interface_write_thread(void *args);

// ------------------------------------------------------------------------------
//   Gimbal Interface Class
// ------------------------------------------------------------------------------
/*
 * Gimbal Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the _messages
 * attribute. The write thread at the moment only streams a heartbeat 1hz
 */
class Gimbal_Interface
{
public:

    /**
     * @brief Messages timestamps
     *
     */
    struct timestamps_t {
        uint64_t heartbeat          = UINT64_MAX;
        uint64_t sys_status         = UINT64_MAX;
        uint64_t mount_status       = UINT64_MAX;
        uint64_t mount_orientation  = UINT64_MAX;
        uint64_t attitude_status    = UINT64_MAX;
        uint64_t raw_imu            = UINT64_MAX;
        uint64_t gimbal_device_info = UINT64_MAX;
        uint64_t command_ack        = UINT64_MAX;
        uint64_t param              = UINT64_MAX;

        void reset_timestamps()
        {
            heartbeat          = UINT64_MAX;
            sys_status         = UINT64_MAX;
            mount_status       = UINT64_MAX;
            mount_orientation  = UINT64_MAX;
            attitude_status    = UINT64_MAX;
            raw_imu            = UINT64_MAX;
            gimbal_device_info = UINT64_MAX;
            command_ack        = UINT64_MAX;
            param              = UINT64_MAX;
        }
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
     * @brief MAVLink Gimbal Protocol
     *
     */
    enum MAVLINK_PROTO {
        MAVLINK_GIMBAL_V1,
        MAVLINK_GIMBAL_V2
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

    enum E_gimbal_axis
    {
        GIMBAL_PITCH_AXIS = 0x00,
        GIMBAL_ROLL_AXIS,
        GIMBAL_YAW_AXIS,
        GIMBAL_TOTAL_AXIS
    };

    struct gimbal_follow_param_t
    {
        float setPoint; /// the new setpoint (angle) each axis of gimbal
        float ratio; /// speed ratio each axis of gimbal
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

    Gimbal_Interface() = delete;

    /**
     * @brief Construct a new Gimbal_Interface object
     *
     * @param serial_port serial object handle low level communication
     * @param sysid mavlink sysid of this system
     * @param compid mavlink compid of this system
     * @param proto MAVLink Gimbal Protocol version
     */
    Gimbal_Interface(Serial_Port *serial_port,
                     uint8_t sysid = 1,
                     uint8_t compid = MAV_COMP_ID_ONBOARD_COMPUTER,
                     MAVLINK_PROTO proto = MAVLINK_GIMBAL_V2);
    ~Gimbal_Interface();

    void start();
    void stop();

    void start_read_thread();
    void start_write_thread(void);

    void handle_quit(int sig);

    bool get_flag_exit(void);

    bool get_connection(void);

    bool present();

    /**
     * @brief  This function reboot the gimbal
     * @param: NONE
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_reboot(void);

    /**
     * @brief  This function set gimbal to rc input mode and block to wait for gimbal response
     * @param: NONE
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_rc_input_sync(void);

    /**
     * @brief  This function set gimbal motor ON/OFF
     * @param: type see control_motor_t
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_motor(control_motor_t type);

    /**
     * @brief  This function get gimbal mode
     * @param: type see gimbal_mode_t
     * @ret: gimbal_mode_t
     */
    Gimbal_Protocol::control_mode_t get_gimbal_mode(void);

    /**
     * @brief  This function reset gimbal with some mode
     * @param: type see gimbal_reset_mode_t
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_reset_mode(Gimbal_Protocol::gimbal_reset_mode_t reset_mode);

    /**
     * @brief  This function set gimbal lock mode
     * @param: NONE
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_lock_mode_sync(void);

    /**
     * @brief  This function set gimbal follow mode
     * @param: NONE
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_follow_mode_sync(void);

    /**
     * @brief  This function rotate to target angle (deg)
     * @param: pitch control pitch value
     * @param: roll control roll value
     * @param: yaw control yaw value (-180; 180)
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_rotation_sync(float pitch, float roll, float yaw);

    /**
     * @brief  This function rotate target rate (deg/s)
     * @param: pitch control pitch value
     * @param: roll control roll value
     * @param: yaw control yaw value
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_rotation_rate_sync(float pitch, float roll, float yaw);

    /**
    * @brief  This function use for move gimbal follow fight controller
    * @param: *pitch is pointer manager follow param of pitch axis of gimbal.
    * @param: *roll is pointer manager follow param of roll axis of gimbal.
    * @param: *yaw is pointer manager follow param of yaw axis of gimbal.
    * @ret: result
    */
    Gimbal_Protocol::result_t set_gimbal_follow_fc_sync(gimbal_follow_param_t *pitch, gimbal_follow_param_t *roll, gimbal_follow_param_t *yaw);

    /**
     * @brief  This function get gimbal status
     * @param: None
     * @ret: Gimbal status
     */
    gimbal_status_t get_gimbal_status(void);

    /**
     * @brief  This function get gimbal imu raw values
     * @param: None
     * @ret: Gimbal raw imu
     */
    imu_t get_gimbal_raw_imu(void);

    /**
     * @brief  This function get gimbal attitude (deg)
     * @param: None
     * @ret: Gimbal attitude
     */
    attitude<float> get_gimbal_attitude(void);

    /**
     * @brief  This function get gimbal encoder depends on encoder type send
     * @param: None
     * @ret: Gimbal encoder
     */
    attitude<int16_t> get_gimbal_encoder(void);

    /**
     * @brief  This function get gimbal timestamps
     * @param: None
     * @ret: Gimbal status
     */
    timestamps_t get_gimbal_timestamps(void);

    /**
     * @brief  This function get the firmware version from gimbal
     * @param: None
     * @ret: see fw_version_t structure
     */
    fw_version_t get_gimbal_version(void)
    {
        fw_version_t fw = { 0 };
        fw.x = _params_list[GMB_PARAM_VERSION_X].value;
        fw.y = _params_list[GMB_PARAM_VERSION_Y].value;
        fw.z = (_params_list[GMB_PARAM_VERSION_Z].value & 0x3F);

        if ((_params_list[GMB_PARAM_VERSION_Z].value & 0xC0) == FIRMWARE_VERSION_TYPE_ALPHA) {
            fw.type = alpha;

        } else if ((_params_list[GMB_PARAM_VERSION_Z].value & 0xC0) == FIRMWARE_VERSION_TYPE_BETA) {
            fw.type = beta;

        } else if ((_params_list[GMB_PARAM_VERSION_Z].value & 0xC0) == FIRMWARE_VERSION_TYPE_RC) {
            fw.type = preview;

        } else if ((_params_list[GMB_PARAM_VERSION_Z].value & 0xC0) == 00) {
            fw.type = official;
        }

        return fw;
    }

    /**
     * @brief  This function shall configure on the tilt axis
     * @param: config see  gimbal_config_axis_t structure
     * @note: The smooth starts with a low value of 50
     *			Slowly increase this setting until you feel an oscillation in the pan axis,
     *			then reduce the setting until the oscillation subsides.
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_config_tilt_axis(const gimbal_config_axis_t &config);

    /**
     * @brief  This function get the config of tilt axis
     * @param: None
     * @ret: gimbal_config_axis_t contains setting related to tilt axis
     */
    gimbal_config_axis_t get_gimbal_config_tilt_axis(void);

    /**
     * @brief  This function shall configure on the roll axis
     * @param: config see  gimbal_config_axis_t structure
     * @note: The smooth starts with a low value of 50
     *			Slowly increase this setting until you feel an oscillation in the pan axis,
     *			then reduce the setting until the oscillation subsides.
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_config_roll_axis(const gimbal_config_axis_t &config);

    /**
     * @brief  This function get the config of roll axis
     * @param: None
     * @ret: gimbal_config_axis_t contains setting related to roll axis
     */
    gimbal_config_axis_t get_gimbal_config_roll_axis(void);

    /**
     * @brief  This function shall configure on the pan axis
     * @param: config see  gimbal_config_axis_t structure
     * @note: The smooth starts with a low value of 50
     *			Slowly increase this setting until you feel an oscillation in the pan axis,
     *			then reduce the setting until the oscillation subsides.
     * @ret: result
     */
    Gimbal_Protocol::result_t set_gimbal_config_pan_axis(const gimbal_config_axis_t &config);

    /**
     * @brief  This function get the config of pan axis
     * @param: None
     * @ret: gimbal_config_axis_t contains setting related to pan axis
     */
    gimbal_config_axis_t get_gimbal_config_pan_axis(void);

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
    Gimbal_Protocol::result_t set_gimbal_motor_control(const gimbal_motor_control_t &tilt,
            const gimbal_motor_control_t &roll,
            const gimbal_motor_control_t &pan,
            uint8_t gyro_filter, uint8_t output_filter, uint8_t gain);

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
    Gimbal_Protocol::result_t get_gimbal_motor_control(gimbal_motor_control_t &tilt,
            gimbal_motor_control_t &roll,
            gimbal_motor_control_t &pan,
            uint8_t &gyro_filter, uint8_t &output_filter, uint8_t &gain);

    /**
     * @brief  This function set the configuration the message mavink with rate
     * @param: encoder_rate - the time rate of the encoder values. Default 10Hz
     * @param: mnt_orient_rate - the time rate of the mount orientation. Default 10Hz
     * @param: attitude_status_rate - the time rate of the attitude status. Default 10Hz
     * @param: raw_imu_rate - the time rate of the raw_imu value. Default 10Hz
     * @NOTE The range [0 - 200Hz]. 0 will disable that message. Only set 1 msg rate higher than 50Hz
     * @ret: None
     */
    Gimbal_Protocol::result_t set_msg_encoder_rate(uint8_t rate);
    Gimbal_Protocol::result_t set_msg_mnt_orient_rate(uint8_t rate);
    Gimbal_Protocol::result_t set_msg_attitude_status_rate(uint8_t rate);
    Gimbal_Protocol::result_t set_msg_raw_imu_rate(uint8_t rate);

    /**
     * @brief Set the gimbal encoder type send
     * @param: true send raw encoder values
     *         false send encoder angle
     * @return result_t
     */
    Gimbal_Protocol::result_t set_gimbal_encoder_type_send(bool type);

    /**
     * @brief reuqest gimbal device info message
     * 
     * @return Gimbal_Protocol::result_t 
     */
    Gimbal_Protocol::result_t request_gimbal_device_info(void);

    /**
     * @brief Get the gimbal encoder type send
     *
     * @return true send raw encoder values
     * @return false send encoder angle
     */
    bool get_gimbal_encoder_type_send(void);

    /**
     * @brief  This function set the enable or disable the reduce drift of the gimbal by using attitude of the aircarf
     * @details Only enable yaw drift if there is an autopilot mount on top the gimbal and send autopilot's attitude
     * to gimbal at rate 50 - 100Hz
     * @param: flag - enable/disable the recude drift of the gimbal by combining attitude from the aircraft
     * @ret: None
     */
    Gimbal_Protocol::result_t set_gimbal_combine_attitude(bool flag);

    /**
     * @brief Set limit angle for pitch.
     * @details Please refer to Gremsy site <gremsy.com> for
     * details about default limit angle of Gimbal.
     * @param limitAngle: limit angle.
     * @return None
     */
    Gimbal_Protocol::result_t set_limit_angle_pitch(const limit_angle_t &limit_angle);

    /**
     * @brief Get limit angle for pitch.
     * @details Please refer to Gremsy site <gremsy.com> for
     * details about default limit angle of Gimbal.
     * @param None
     * @return limit angle
     */
    limit_angle_t get_limit_angle_pitch(void);

    /**
     * @brief Set limit angle for yaw.
     * @details Please refer to Gremsy site <gremsy.com> for
     * details about default limit angle of Gimbal.
     * @param limitAngle: limit angle.
     * @return None
     */
    Gimbal_Protocol::result_t set_limit_angle_yaw(const limit_angle_t &limit_angle);

    /**
     * @brief Get limit angle for yaw.
     * @details Please refer to Gremsy site <gremsy.com> for
     * details about default limit angle of Gimbal.
     * @param None
     * @return limit angle
     */
    limit_angle_t get_limit_angle_yaw(void);

    /**
     * @brief Set limit angle for roll.
     * @details Please refer to Gremsy site <gremsy.com> for
     * details about default limit angle of Gimbal.
     * @param limitAngle: limit angle.
     * @return None
     */
    Gimbal_Protocol::result_t set_limit_angle_roll(const limit_angle_t &limit_angle);

    /**
     * @brief Get limit angle for roll.
     * @details Please refer to Gremsy site <gremsy.com> for
     * details about default limit angle of Gimbal.
     * @param None
     * @return limit angle
     */
    limit_angle_t get_limit_angle_roll(void);

    /**
     * @brief Set the external RC type
     * @param type rc type
     * 
     * @return Gimbal_Protocol::result_t 
     */
    Gimbal_Protocol::result_t set_rc_type(rc_type_t type);

private:

    /**
     * @brief Gimbal Interface state
     *
     */
    enum interface_state_t {
        GIMBAL_STATE_NOT_PRESENT = 0,
        GIMBAL_STATE_PRESENT_INITIALIZING,
        GIMBAL_STATE_PRESENT_IDLING,
        GIMBAL_STATE_PRESENT_ALIGNING,
        GIMBAL_STATE_PRESENT_RUNNING
    };

    /**
     * @brief MAVLink messages used in interface
     *
     */
    struct messages_t {
        pthread_mutex_t mutex;

        // Heartbeat
        mavlink_heartbeat_t heartbeat = { 0 };

        // System Status
        mavlink_sys_status_t sys_status = { 0 };

        // Mount status contains the encoder count value. Resolution 2^16
        mavlink_mount_status_t mount_status = { 0 };

        // Mount orientation
        mavlink_mount_orientation_t mount_orientation = { 0 };

        // Gimbal Attitude status
        mavlink_gimbal_device_attitude_status_t atttitude_status = { 0 };

        // Gimbal device info
        mavlink_gimbal_device_information_t gimbal_device_info = { 0 };

        // Raw IMU
        mavlink_raw_imu_t raw_imu = { 0 };

        // timestamps
        timestamps_t timestamps;

        messages_t()
        {
            pthread_mutex_init(&mutex, NULL);
        }

        ~messages_t()
        {
            pthread_mutex_destroy(&mutex);
        }

        void reset_timestamps()
        {
            timestamps.reset_timestamps();
        }
    };

    enum serial_thread_status_t {
        THREAD_NOT_INIT = 0,
        THREAD_RUNNING,
        THREAD_IDLING,        
    };

    /**
     * @brief Gimbal's sensor status
     */
    enum sensor_state_t {
        SENSOR_OK        = 0x00,   /* Gimbal's sensor is healthy */
        SENSOR_IMU_ERROR = 0x01,   /* IMU error*/
        SENSOR_EN_TILT   = 0x02,   /* Encoder sensor is error at tilt axis*/
        SENSOR_EN_ROLL   = 0x03,   /* Encoder sensor is error at roll axis*/
        SENSOR_EN_PAN    = 0x04,   /* Encoder sensor is error at pan axis*/
    };

    /**
     * @brief param_state_t
     * Status of param process
     */
    enum param_state_t {
        PARAM_STATE_NOT_YET_READ      = 0,   // parameter has yet to be initialized
        PARAM_STATE_FETCH_AGAIN       = 1,   // parameter is being fetched
        PARAM_STATE_ATTEMPTING_TO_SET = 2,   // parameter is being set
        PARAM_STATE_CONSISTENT        = 3,   // parameter is consistent
        PARAM_STATE_NONEXISTANT       = 4    // parameter does not seem to exist
    };

    /**
     * @brief param_index_t
     * Gimbal opens some parameters for setting. Please refer to user manual to learn more how to set
     * that parameters
     */
    enum param_index_t {
        GMB_PARAM_VERSION_X = 0,
        GMB_PARAM_VERSION_Y,
        GMB_PARAM_VERSION_Z,

        GMB_PARAM_STIFFNESS_PITCH,
        GMB_PARAM_STIFFNESS_ROLL,
        GMB_PARAM_STIFFNESS_YAW,

        GMB_PARAM_HOLDSTRENGTH_PITCH,
        GMB_PARAM_HOLDSTRENGTH_ROLL,
        GMB_PARAM_HOLDSTRENGTH_YAW,

        GMB_PARAM_OUTPUT_FILTER,
        GMB_PARAM_GYRO_FILTER,
        GMB_PARAM_GAIN,

        GMB_PARAM_SPEED_FOLLOW_PITCH,
        GMB_PARAM_SPEED_FOLLOW_YAW,

        GMB_PARAM_SMOOTH_FOLLOW_PITCH,
        GMB_PARAM_SMOOTH_FOLLOW_YAW,

        GMB_PARAM_WINDOW_FOLLOW_PITCH,
        GMB_PARAM_WINDOW_FOLLOW_YAW,

        GMB_PARAM_SPEED_CONTROL_PITCH,
        GMB_PARAM_SPEED_CONTROL_ROLL,
        GMB_PARAM_SPEED_CONTROL_YAW,

        GMB_PARAM_SMOOTH_CONTROL_PITCH,
        GMB_PARAM_SMOOTH_CONTROL_ROLL,
        GMB_PARAM_SMOOTH_CONTROL_YAW,

        GMB_PARAM_AXIS_DIR,

        GMB_PARAM_ENCODER_TYPE,

        GMB_PARAM_MIN_LIMIT_ANGLE_PITCH,
        GMB_PARAM_MAX_LIMIT_ANGLE_PITCH,
        GMB_PARAM_MIN_LIMIT_ANGLE_ROLL,
        GMB_PARAM_MAX_LIMIT_ANGLE_ROLL,
        GMB_PARAM_MIN_LIMIT_ANGLE_YAW,
        GMB_PARAM_MAX_LIMIT_ANGLE_YAW,

        GMB_PARAM_RC_TYPE,

        GIMBAL_NUM_TRACKED_PARAMS
    };

    Serial_Port *_serial_port;

    Gimbal_Protocol *_gimbal_proto;
    MAVLINK_PROTO _proto;

    mavlink_system_t _system;
    mavlink_system_t _gimbal;

    bool time_to_exit = false;
    bool has_detected = false;

    pthread_t read_tid  = 0;
    pthread_t write_tid = 0;

    volatile serial_thread_status_t reading_status = THREAD_NOT_INIT;
    volatile serial_thread_status_t writing_status = THREAD_NOT_INIT;

    void messages_handler(const mavlink_message_t &message);
    int write_message(const mavlink_message_t &message);

    void read_thread(void);
    void write_thread(void);

    void write_heartbeat(void);

    messages_t _messages;

    gimbal_status_t _status;

    interface_state_t _state = GIMBAL_STATE_NOT_PRESENT;

    static constexpr char *alpha    = "ALPHA";
    static constexpr char *beta     = "BETA";
    static constexpr char *preview  = "PREVIEW";
    static constexpr char *official = "OFFICIAL";

    /**
     * @brief Function check if compid is gimbal
     *
     * @param compid
     * @return true compid is gimbal
     * @return false
     */
    bool is_gimbal(uint8_t compid);

    /**
     * @brief Set the msg rate
     *
     * @param msgid
     * @param rate
     * @return Gimbal_Protocol::result_t
     */
    Gimbal_Protocol::result_t set_msg_rate(uint32_t msgid, uint8_t rate);

    /**
     * @brief Request msg from gimbal
     *
     * @param msgid
     * @return result_t
     */
    Gimbal_Protocol::result_t request_msg(uint32_t msgid);

    // Gimbal params
    void reset_params();
    bool params_initialized();
    bool params_received_all();
    void fetch_params();

    void param_update();
    void param_process(void);

    const char *get_param_name(param_index_t param)
    {
        return _params_list[param].gmb_id;
    }
    const uint8_t get_gmb_index(param_index_t param)
    {
        return _params_list[param].gmb_idx;
    }

    Gimbal_Protocol::result_t request_param(param_index_t param);
    Gimbal_Protocol::result_t request_param_list(void);

    /**
     * @brief  This function for user to get gimbal param
     *
     * @param: param - param name to get, follow param_index_t
     * @param: value - reference to value
     * @ret: None
     */
    Gimbal_Protocol::result_t get_param(param_index_t param, int16_t &value);

    /**
     * @brief  This function for user to set gimbal param
     *
     * @param: param - param name to set, follow param_index_t
     * @param: value - value to set
     * @ret: None
     */
    Gimbal_Protocol::result_t set_param(param_index_t param, int16_t value);

    static constexpr uint32_t _TIME_LOST_CONNECT = 60000000;  // 60s
    static constexpr uint32_t _RETRY_PERIOD      = 100;       // 100ms
    static constexpr uint8_t _MAX_FETCH_TIME     = 5;         // times

    struct {
        const uint8_t gmb_idx;
        const char *gmb_id;
        int16_t value;

        volatile param_state_t state;
        uint8_t fetch_attempts;
        bool seen;
    } _params_list[GIMBAL_NUM_TRACKED_PARAMS] = {

        // Gimbal version
        { 0, "VERSION_X", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 67, "VERSION_Y", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 68, "VERSION_Z", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        // Gimbal stiffness
        { 2, "PITCH_P", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 5, "ROLL_P", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 8, "YAW_P", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        // Gimbal hold strength
        { 11, "PITCH_POWER", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 12, "ROLL_POWER", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 13, "YAW_POWER", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        { 9, "YAW_I", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 29, "GYRO_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 3, "PITCH_I", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        // Gimbal speed follow
        { 14, "PITCH_FOLLOW", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 16, "YAW_FOLLOW", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        // Gimbal follow filter
        { 17, "PITCH_FILTER", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 19, "YAW_FILTER", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        // Gimbal follow windown
        { 57, "TILT_WINDOW", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 58, "PAN_WINDOW", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        // Gimbal speed control
        { 60, "RC_PITCH_SPEED", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 61, "RC_ROLL_SPEED", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 62, "RC_YAW_SPEED", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        // Gimbal smooth control
        { 36, "RC_PITCH_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 37, "RC_ROLL_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 38, "RC_YAW_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        // Direction
        { 63, "JOY_AXIS", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        { 75, "ENC_TYPE_SEND", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        { 30, "TRAVEL_MIN_PIT", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 31, "TRAVEL_MAX_PIT", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 32, "TRAVEL_MIN_ROLL", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 33, "TRAVEL_MAX_ROLL", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 69, "TRAVEL_MIN_PAN", 0, PARAM_STATE_NOT_YET_READ, 0, false },
        { 70, "TRAVEL_MAX_PAN", 0, PARAM_STATE_NOT_YET_READ, 0, false },

        { 28, "RADIO_TYPE", 0, PARAM_STATE_NOT_YET_READ, 0, false },
    };

    uint64_t _last_request_ms;
    uint64_t _last_set_ms;
};

#endif // GIMBAL_INTERFACE_H_

/************************ (C) COPYRIGHT Gremsy *****END OF FILE****************/
