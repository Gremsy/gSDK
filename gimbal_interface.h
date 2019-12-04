/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    gimbal_interface.h
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-021-2018
 * @brief   This file contains expand of gMavlink
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
void* start_gimbal_interface_read_thread(void *args);
void* start_gimbal_interface_write_thread(void *args);


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

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

	// Mount status
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
typedef enum _gimbal_state
{
    GIMBAL_STATE_OFF            = 0x00,     /*< Gimbal is off*/
    GIMBAL_STATE_INIT           = 0x01,     /*< Gimbal is initializing*/
    GIMBAL_STATE_ON             = 0x02,     /*< Gimbal is on */
    GIMBAL_STATE_LOCK_MODE      = 0x04,     
    GIMBAL_STATE_FOLLOW_MODE    = 0x08,
    GIMBAL_STATE_SEARCH_HOME    = 0x10,
    GIMBAL_STATE_SET_HOME       = 0x20,
    GIMBAL_STATE_ERROR          = 0x40
} gimbal_state_t;

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


typedef enum _version
{
	VERSION_OFFICAL = 0x00,
    VERSION_ALPHA   = 0x01,
    VERSION_BETA    = 0x02,
    VERSION_PREVIEW = 0x03,
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

    int gimbal_set_angle(int16_t tilt, int16_t roll, int16_t pan);

    bool get_connection(void);

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
    void set_gimbal_move(int16_t tilt, int16_t roll, int16_t pan);


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
private:

	Serial_Port *serial_port;

	bool time_to_exit;
	bool has_detected;

	pthread_t read_tid;
	pthread_t write_tid;

	void read_thread();
	void write_thread(void);

	void write_setpoint();
	void write_heartbeat(void);

	Mavlink_Messages current_messages;

	gimbal_status_t gimbal_status;
};


#endif // GIMBAL_INTERFACE_H_

/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.*****END OF FILE**********/