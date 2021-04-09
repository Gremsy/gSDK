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

/* Includes ------------------------------------------------------------------*/
#ifndef GIMBAL_INTERFACE_H_
#define GIMBAL_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"

#include <signal.h>
#include <time.h>
#include <sys/time.h>

// #include <common/mavlink.h>
#include <ardupilotmega/mavlink.h>

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------
#define SYSID_ONBOARD                       4

#ifndef PI
#define PI          3.141592654
#endif

#define PI2ANGLE    (180.0/PI)

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
uint64_t get_time_usec();
uint64_t get_time_msec();
void* start_gimbal_interface_read_thread(void *args);
void* start_gimbal_interface_write_thread(void *args);


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

enum gimbal_state_t {
    GIMBAL_STATE_NOT_PRESENT = 0,
    GIMBAL_STATE_PRESENT_INITIALIZING,
    GIMBAL_STATE_PRESENT_ALIGNING,
    GIMBAL_STATE_PRESENT_RUNNING
};

struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t mount_status;
	uint64_t mount_orientation;
	uint64_t raw_imu;
	uint64_t command_ack;

	void
	reset_timestamps()
	{
		heartbeat 			= 0;
		sys_status 			= 0;
		mount_status 		= 0;
		mount_orientation 	= 0;
		raw_imu 			= 0;
		command_ack 		= 0;
	}

};

struct Sequence_Numbers
{
	Sequence_Numbers()
	{
		reset_seq_num();
	}

	uint8_t heartbeat;
	uint8_t sys_status;
	uint8_t mount_status;
	uint8_t mount_orientation;
	uint8_t raw_imu;
	uint8_t command_ack;

	void reset_seq_num()
	{
		heartbeat 				= 0;
		sys_status 				= 0;
		mount_status 			= 0;
		mount_orientation 		= 0;
		raw_imu 				= 0;
		command_ack 			= 0;
	}
};  


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages 
{

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t 		heartbeat;

	// System Status
	mavlink_sys_status_t 		sys_status;

	// Mount status contains the encoder count value. Resolution 2^16
	mavlink_mount_status_t 		mount_status;

	// Mount orientation
	mavlink_mount_orientation_t mount_orientation;

	// Attitude
	mavlink_raw_imu_t 			raw_imu;

	// Time Stamps
	Time_Stamps time_stamps;

	// Command acknowledgement. MAV_CMD_DO_MOUNT_CONFIGURE
	uint8_t result_cmd_ack_msg_configure;

	// Command acknowledgement. MAV_CMD_DO_MOUNT_CONTROL
	uint8_t result_cmd_ack_msg_control;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

	// Sequence number of last packet received
	Sequence_Numbers current_seq_rx;

	void 
	reset_seq_num()
	{
		current_seq_rx.reset_seq_num();
	}
};


/**
 * @brief control_motor_t
 * Command control motor is on/off
 */
typedef enum _control_direction
{
	DIR_CW 	= 0x00,
	DIR_CCW = 0x01
} control_gimbal_direction_t;

/**
 * @brief control_motor_t
 * Command control motor is on/off
 */
typedef enum _control_gimbal_motor_t
{
    TURN_OFF    = 0x00,
    TURN_ON     = 0x01
} control_gimbal_motor_t;

/**
 * @brief control_mode_t
 * Command control gimbal mode lock/follow
 */
typedef enum _control_gimbal_mode_t
{
	GIMBAL_OFF  = 0x00,
    LOCK_MODE   = 0x01,
    FOLLOW_MODE = 0x02,
} control_gimbal_mode_t;

/**
 * @brief _control_gimbal_axis_input_mode
 * Command control gimbal input mode for each axis
 */
typedef enum _control_gimbal_axis_input_mode
{
    CTRL_ANGLE_BODY_FRAME       = 0,
    CTRL_ANGULAR_RATE           = 1,
    CTRL_ANGLE_ABSOLUTE_FRAME   = 2,
} control_gimbal_axis_input_mode_t;

/**
 * @brief gimbal_state_t
 * State of gimbal
 */
typedef enum 
{
    GIMBAL_STATE_OFF            = 0x00,     /*< Gimbal is off*/
    GIMBAL_STATE_INIT           = 0x01,     /*< Gimbal is initializing*/
    GIMBAL_STATE_ON             = 0x02,     /*< Gimbal is on */
    GIMBAL_STATE_LOCK_MODE      = 0x04,     
    GIMBAL_STATE_FOLLOW_MODE    = 0x08,
    GIMBAL_STATE_SEARCH_HOME    = 0x10,
    GIMBAL_STATE_SET_HOME       = 0x20,
    GIMBAL_STATE_ERROR          = 0x40
} gimbal_state_operation_t;

/**
 * @brief gimbal_state_t
 * State of gimbal's sensor
 */
typedef enum _sensor_state
{
    SENSOR_OK                   = 0x00,     /* Gimbal's sensor is healthy */
    SENSOR_IMU_ERROR            = 0x01,     /* IMU error*/
    SENSOR_EN_TILT              = 0x02,     /* Encoder sensor is error at tilt axis*/
    SENSOR_EN_ROLL              = 0x03,     /* Encoder sensor is error at roll axis*/
    SENSOR_EN_PAN               = 0x04,     /* Encoder sensor is error at pan axis*/
} sensor_state_;


typedef struct _version
{
	uint8_t x;
	uint8_t y;
	uint8_t z;
	const char* type;
} fw_version_t;


/**
 * @brief _control_gimbal_axis_mode_t
 * Command control gimbal for each axis
 */
typedef struct _control_gimbal_axis_mode_t
{
    /* stabilize? (1 = yes, 0 = no)*/
    uint8_t stabilize;   
    
    control_gimbal_axis_input_mode_t    input_mode;
    
}control_gimbal_axis_mode_t;


/**
 * @brief gimbal_state_t
 * State of gimbal's sensor
 */
typedef struct _gimbal_status_t
{
    uint16_t    load; /*< [ms] Maximum usage the mainloop time. Values: [0-1000] - should always be below 1000*/
    uint16_t    voltage_battery; /*< [V] Battery voltage*/
    uint8_t     sensor; /*< Specific sensor occur error (encorder, imu) refer sensor_state_*/
    uint16_t    state;  /* System state of gimbal. Refer gimbal_state_t*/
    uint8_t     mode;   /*< Gimbal mode is running*/
    uint32_t    seq;
} gimbal_status_t;


/**
 * @brief _gimbal_config_t
 * This structure will contain the gimbal configuration related to speed, smooth, direction
 */
typedef struct _gimbal_config_axis_t
{
	int8_t 	dir;
	uint8_t	speed_control;
	uint8_t smooth_control;

	uint8_t speed_follow;
	uint8_t smooth_follow;
	uint8_t	window_follow;

} gimbal_config_axis_t;


/**
 * @brief config_mavlink_message_t
 * This structure will contain the configuration related to mavlink message
 */
typedef struct _config_mavlink_message_t
{
	uint8_t emit_heatbeat;
	uint8_t status_rate;
	uint8_t enc_value_rate; 
	uint8_t enc_type_send;
	uint8_t orientation_rate;
	uint8_t imu_rate;
	
} config_mavlink_message_t;


/**
 * @brief _gimbal_motor_control_t
 * stifness: Stiffness setting has a significant impact on the performance of the Gimbal. 
 *			This setting adjusts the degrees to which the gimbal tries to correct 
 *			for unwanted camera movement and hold the camera stable. 
 * 			The higher you can run the setting without vibration or oscillation, the better.
 * Holdstrength: Power level required for the corresponding axis. 
 *				This option is only recommended for advanced users. Set 40 as defaults
 */
typedef struct _gimbal_motor_control_t
{
	uint8_t stiffness;
	uint8_t	holdstrength;
} gimbal_motor_control_t;


	/**
	 * @brief control_motor_t
	 * Command control motor is on/off
	 */
	enum param_state_t
	{
		PARAM_STATE_NOT_YET_READ 		= 0,	// parameter has yet to be initialized
		PARAM_STATE_FETCH_AGAIN			= 1,	// parameter is being fetched
		PARAM_STATE_ATTEMPTING_TO_SET   = 2,	// parameter is being set
		PARAM_STATE_CONSISTENT			= 3,	// parameter is consistent
		PARAM_STATE_NONEXISTANT			= 4		// parameter does not seem to exist
	};


	/**
	 * @brief param_index_t
	 * Gimbal opens some parameters for setting. Please refer to user manual to learn more how to set 
	 * that parameters
	 */
	enum param_index_t
	{

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

		GMB_PARAM_HEATBEAT_EMIT,
		GMB_PARAM_STATUS_RATE,
		GMB_PARAM_ENCODER_VALUE_RATE,
		GMB_PARAM_ENCODER_TYPE,
		GMB_PARAM_ORIENTATION_RATE,
		GMB_PARAM_RAW_IMU_RATE,

	   GIMBAL_NUM_TRACKED_PARAMS
	};
// ----------------------------------------------------------------------------------
//   Gimbal Interface Class
// ----------------------------------------------------------------------------------
/*
 * Gimbal Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the current_messages
 * attribute.  The write thread at the moment only streams a heartbeat 1hz
 */
class Gimbal_Interface
{

public:

	Gimbal_Interface();
	Gimbal_Interface(Serial_Port *serial_port_);
	~Gimbal_Interface();

	char reading_status;
	char writing_status;
    uint64_t write_count;

    /// Id of gimbal if it has mounted
    int system_id;
	int gimbal_id;
	int companion_id;

	void read_messages();
	int  write_message(mavlink_message_t message);

	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);

	void handle_quit( int sig );

	bool get_flag_exit(void);

    bool get_connection(void);

    bool present();
    /**
	 * @brief  This function shall reboot the gimbal
	 * @param: NONE
	 * @ret: None
	 */
    void set_gimbal_reboot(void);

	/**
	 * @brief  This function shall turn on/off gimbal
	 * @param: type see control_gimbal_motor_t
	 * @ret: None
	 */
    void set_gimbal_motor_mode(control_gimbal_motor_t type);

    /**
	 * @brief  This function shall set gimbal mode
	 * @param: type see control_gimbal_mode_t
	 * @ret: None
	 */
    void set_gimbal_mode(control_gimbal_mode_t mode);


     /**
	 * @brief  This function shall set mode for each axis
	 * @param: type see control_gimbal_axis_mode_t
	 * @ret: None
	 */
    void set_gimbal_axes_mode(control_gimbal_axis_mode_t tilt,
                        control_gimbal_axis_mode_t roll,
                        control_gimbal_axis_mode_t pan);

    /**
	 * @brief  This function shall set mode for each axis.
	 * The gimbal will move following the gimbal axes mode.
	 * @param: type see control_gimbal_axis_mode_t
	 * @ret: None
	 */
    void set_gimbal_move(float tilt, float roll, float pan);

    /**
	 * @brief  This function get gimbal status
	 * @param: None
	 * @ret: Gimbal status
	 */
    gimbal_status_t get_gimbal_status(void);

    /**
	 * @brief  This function get gimbal imu raw values
	 * @param: None
	 * @ret: Gimbal status
	 */
    mavlink_raw_imu_t get_gimbal_raw_imu(void);

    /**
	 * @brief  This function get gimbal mount orientation
	 * @param: None
	 * @ret: Gimbal status
	 */
    mavlink_mount_orientation_t get_gimbal_mount_orientation(void);

        /**
	 * @brief  This function get gimbal mount status
	 * @param: None
	 * @ret: Gimbal status
	 */
    mavlink_mount_status_t get_gimbal_mount_status(void);

	/**
	 * @brief  This function get gimbal time stamps 
	 * @param: None
	 * @ret: Gimbal status
	 */
	Time_Stamps get_gimbal_time_stamps(void);

	/**
	 * @brief  This function get gimbal the sequence numbers 
	 * @param: None
	 * @ret: Gimbal status
	 */
	Sequence_Numbers get_gimbal_seq_num(void);

	/**
	 * @brief  This function get the feedback from gimbal after sending 
	 * MAV_CMD_DO_MOUNT_CONFUGURE
	 * @param: None
	 * @ret: In-progress or Accepted. Refer to @MAV_RESULT
	 */

	uint8_t get_command_ack_do_mount_configure(void);

	/**
	 * @brief  This function get the feedback from gimbal after sending 
	 * MAV_CMD_DO_MOUNT_CONTROL
	 * @param: None
	 * @ret: In-progress or Accepted. Refer to @MAV_RESULT
	 */

	uint8_t get_command_ack_do_mount_control(void);

 	/**
	 * @brief  This function get the firmware version from gimbal
	 * 
	 * @param: None
	 * @ret: see fw_version_t structure
	 */
    fw_version_t get_gimbal_version(void)
    {
    	fw_version_t fw;

		fw.x = 	_params_list[GMB_PARAM_VERSION_X].value;
		fw.y = 	_params_list[GMB_PARAM_VERSION_Y].value;
		fw.z = 	(_params_list[GMB_PARAM_VERSION_Z].value & 0x3F);

		if((_params_list[GMB_PARAM_VERSION_Z].value & 0xC0) == FIRMWARE_VERSION_TYPE_ALPHA)
		{
			fw.type = this->alpha;
		}
		else if((_params_list[GMB_PARAM_VERSION_Z].value & 0xC0) == FIRMWARE_VERSION_TYPE_BETA)
		{
			fw.type = this->beta;
		}
		else if((_params_list[GMB_PARAM_VERSION_Z].value & 0xC0) == FIRMWARE_VERSION_TYPE_RC)
		{
			fw.type = this->preview;
		}
		else if((_params_list[GMB_PARAM_VERSION_Z].value & 0xC0) == 00)
		{
			fw.type = this->official;
		}

    	return fw;
    }

    /**
	 * @brief  This function shall configure on the tilt axis
	 * 
	 * @param: config see  gimbal_config_axis_t structure
	 * @note: The smooth starts with a low value of 50 
	 *			Slowly increase this setting until you feel an oscillation in the pan axis, 
	 *			then reduce the setting until the oscillation subsides.
	 * @ret: None
	 */
	void set_gimbal_config_tilt_axis(gimbal_config_axis_t config);

	/**
	 * @brief  This function get the config of tilt axis
	 * 
	 * @param: None
	 * @ret: gimbal_config_axis_t contains setting related to tilt axis
	 */
	gimbal_config_axis_t get_gimbal_config_tilt_axis(void);

	/**
	 * @brief  This function shall configure on the roll axis
	 * 
	 * @param: config see  gimbal_config_axis_t structure
	 * @note: The smooth starts with a low value of 50 
	 *			Slowly increase this setting until you feel an oscillation in the pan axis, 
	 *			then reduce the setting until the oscillation subsides.
	 * @ret: None
	 */
	void set_gimbal_config_roll_axis(gimbal_config_axis_t config);

	/**
	 * @brief  This function get the config of roll axis
	 * 
	 * @param: None
	 * @ret: gimbal_config_axis_t contains setting related to roll axis
	 */
	gimbal_config_axis_t get_gimbal_config_roll_axis(void);

	/**
	 * @brief  This function shall configure on the pan axis
	 * 
	 * @param: config see  gimbal_config_axis_t structure
	 * @note: The smooth starts with a low value of 50 
	 *			Slowly increase this setting until you feel an oscillation in the pan axis, 
	 *			then reduce the setting until the oscillation subsides.
	 * @ret: None
	 */
	void set_gimbal_config_pan_axis(gimbal_config_axis_t config);

	/**
	 * @brief  This function get the config of pan axis
	 * 
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
	void set_gimbal_motor_control(	gimbal_motor_control_t tilt, 
									gimbal_motor_control_t roll,
									gimbal_motor_control_t pan, 
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
	void get_gimbal_motor_control(	gimbal_motor_control_t& tilt, 
									gimbal_motor_control_t& roll,
									gimbal_motor_control_t& pan, 
									uint8_t& gyro_filter, uint8_t& output_filter, uint8_t& gain);



	/**
	 * @brief  This function set the configuration the message mavink with rate 
	 * 
	 * @param: emit_heatbeat - enable the heartbeat when lost connection or not enable = 1, disable = 0
	 * @param: status_rate - the time rate of the system status. Gimbal sends as default 10Hz
	 * @param: enc_value_rate - the time rate of the encoder values. Gimbal sends as default 50Hz
	 * @param: enc_type_send - Set the type of encoder has been sent from gimbal is angle or count (Resolution 2^16)
	 * @param: orien_rate - the time rate of the mount orientation of gimbal.Gimbal sends as default 50Hz
	 * @param: imu_rate - the time rate of the raw_imu value. Gimbal sends as default 10Hz
	 * @NOTE The range [0 - 100Hz]. 0 will disable that message
	 * @ret: None
	 */

	void set_gimbal_config_mavlink_msg(uint8_t emit_heatbeat = 1, 
										uint8_t status_rate = 10, 
										uint8_t enc_value_rate = 50, 
										uint8_t enc_type_send = 0,
										uint8_t orien_rate = 50,
										uint8_t imu_rate = 10);

	/**
	 * @brief  This function get the mavlink configuration message
	 * 
	 * @param: None
	 * @ret: config_mavlink_message_t contains setting related to mavlink configuration message
	 */
	config_mavlink_message_t get_gimbal_config_mavlink_msg(void);

	/**
	 * @brief  This function for user to get gimbal param
	 * 
	 * @param: param - param name to get, follow param_index_t
	 * @param: value - The coefficent for denoising the param value
	 * @param: def_value - The coefficent for denoising the param def_value
	 * @ret: None
	 */
	void get_param(param_index_t param, int16_t& value, int16_t def_value = 0);

	/**
	 * @brief  This function for user to set gimbal param
	 * 
	 * @param: param - param name to get, follow param_index_t
	 * @param: value - value to set
	 * @ret: None
	 */
	void set_param(param_index_t param, int16_t value);

private:

	Serial_Port *serial_port;

	bool time_to_exit;
	bool has_detected;
	uint32_t _last_report_msg_us;

	pthread_t read_tid;
	pthread_t write_tid;

	void read_thread();
	void write_thread(void);

	void write_setpoint();
	void write_heartbeat(void);

	Mavlink_Messages current_messages;

	gimbal_status_t gimbal_status;

    gimbal_state_t _state;

	constexpr static const char* alpha  		= "ALPHA";
	constexpr static const char* beta    	 	= "BETA";
	constexpr static const char* preview    	= "PREVIEW";
	constexpr static const char* official   	= "OFFICIAL";

	//Gimbal params
	void reset_params();
	bool params_initialized();
	bool params_received_all();
	void fetch_params();

	void param_update();
    void param_process(void);
	void handle_param_value(mavlink_message_t *msg);

	const char* get_param_name(param_index_t param)
	{
		return _params_list[param].gmb_id;
	}
	const uint8_t get_gmb_index(param_index_t param)
	{
		return _params_list[param].gmb_idx;
	}

	const uint32_t	_time_lost_connection = 60000000;
	const uint32_t 	_retry_period	= 100;  //100ms
	const uint8_t 	_max_fetch_attempts = 5; // times

	struct 
	{
		const uint8_t gmb_idx;
		const char* gmb_id;
		int16_t value;

		param_state_t state;
		uint8_t	fetch_attempts;
		bool seen;

	} _params_list[GIMBAL_NUM_TRACKED_PARAMS] = {

		// Gimbal version
		{0, "VERSION_X", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{67, "VERSION_Y", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{68, "VERSION_Z", 0, PARAM_STATE_NOT_YET_READ, 0, false},

		// Gimbal stiffness
		{2, "PITCH_P", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{5, "ROLL_P", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{8, "YAW_P", 0, PARAM_STATE_NOT_YET_READ, 0, false},

		// Gimbal hold strength
		{11, "PITCH_POWER", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{12, "ROLL_POWER", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{13, "YAW_POWER", 0, PARAM_STATE_NOT_YET_READ, 0, false},

		{9, "YAW_I", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{29, "GYRO_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{3, "PITCH_I", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		

		// Gimbal speed follow
		{14, "PITCH_FOLLOW", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{16, "YAW_FOLLOW", 0, PARAM_STATE_NOT_YET_READ, 0, false},

		// Gimbal follow filter
		{17, "PITCH_FILTER", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{19, "YAW_FILTER", 0, PARAM_STATE_NOT_YET_READ, 0, false},

		// Gimbal follow windown
		{57, "TILT_WINDOW", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{58, "PAN_WINDOW", 0, PARAM_STATE_NOT_YET_READ, 0, false},

		// Gimbal speed control
		{60, "RC_PITCH_SPEED", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{61, "RC_ROLL_SPEED", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{62, "RC_YAW_SPEED", 0, PARAM_STATE_NOT_YET_READ, 0, false},

		// Gimbal smooth control
		{36, "RC_PITCH_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{37, "RC_ROLL_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{38, "RC_YAW_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false},

		// Direction
		{63, "JOY_AXIS", 0, PARAM_STATE_NOT_YET_READ, 0, false},

		// Setting message rate
		{72, "HEARTBEAT_EMIT", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{73, "STATUS_RATE", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{74, "ENC_CNT_RATE", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{75, "ENC_TYPE_SEND", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{76, "ORIEN_RATE", 0, PARAM_STATE_NOT_YET_READ, 0, false},
		{77, "IMU_RATE", 0, PARAM_STATE_NOT_YET_READ, 0, false},

	};

	uint64_t _last_request_ms;
};


#endif // GIMBAL_INTERFACE_H_

/************************ (C) COPYRIGHT Gremsy *****END OF FILE****************/
