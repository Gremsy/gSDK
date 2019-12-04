/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    gimbal_interface.c
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-021-2018
 * @brief   This file contains expand of hal_dma
 *
 ******************************************************************************/
 
/* Includes ------------------------------------------------------------------*/

#include "gimbal_interface.h"

// Gimbal status 1
typedef enum
{
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
} status1_t;

// Gimbal status 2
typedef enum
{
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
} status2_t;

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Gimbal_Interface::
Gimbal_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	time_to_exit   = false;  // flag to signal thread exit
	has_detected   = false;	// Flag to detect gimbal

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	gimbal_id 	 = MAV_COMP_ID_GIMBAL; // gimbal component id
	companion_id = MAV_COMP_ID_SYSTEM_CONTROL; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = gimbal_id;

	serial_port = serial_port_; // serial port management object

}

Gimbal_Interface::
~Gimbal_Interface()
{}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Gimbal_Interface::
read_messages()
{
	bool success;               // receive success flag
	Time_Stamps 			this_timestamps;
	Sequence_Numbers 		this_seq_num;

	// Blocking wait for new data
	while (!time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{
			// Handle Message ID
			switch (message.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;

					if(has_detected == false)
					{
						// Store message sysid and compid.
						// Note this doesn't handle multiple message sources.
						current_messages.sysid  = message.sysid;
						current_messages.compid = message.compid;

						has_detected = true;
					}

					mavlink_status_t    *chan_status = mavlink_get_channel_status(MAVLINK_COMM_1);
					this_seq_num.heartbeat = chan_status->current_rx_seq;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					// printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;

					mavlink_status_t    *chan_status = mavlink_get_channel_status(MAVLINK_COMM_1);
					this_seq_num.sys_status = chan_status->current_rx_seq;
					break;
				}

				case MAVLINK_MSG_ID_MOUNT_STATUS:
				{
					// printf("MAVLINK_MSG_ID_MOUNT_STATUS\n");
					mavlink_msg_mount_status_decode(&message, &(current_messages.mount_status));
					current_messages.time_stamps.mount_status = get_time_usec();
					this_timestamps.mount_status = current_messages.time_stamps.mount_status;

					mavlink_status_t    *chan_status = mavlink_get_channel_status(MAVLINK_COMM_1);
					this_seq_num.mount_status = chan_status->current_rx_seq;
					break;
				}

				case MAVLINK_MSG_ID_MOUNT_ORIENTATION:
				{
					// printf("MAVLINK_MSG_ID_MOUNT_ORIENTATION\n");
					mavlink_msg_mount_orientation_decode(&message, &(current_messages.mount_orientation));
					current_messages.time_stamps.mount_orientation = get_time_usec();
					this_timestamps.mount_orientation = current_messages.time_stamps.mount_orientation;

					mavlink_status_t    *chan_status = mavlink_get_channel_status(MAVLINK_COMM_1);
					this_seq_num.mount_orientation = chan_status->current_rx_seq;
					break;
				}

				case MAVLINK_MSG_ID_RAW_IMU:
				{
					// printf("MAVLINK_MSG_ID_RAW_IMU\n");
					mavlink_msg_raw_imu_decode(&message, &(current_messages.raw_imu));
					current_messages.time_stamps.raw_imu = get_time_usec();
					this_timestamps.raw_imu = current_messages.time_stamps.raw_imu;

					mavlink_status_t    *chan_status = mavlink_get_channel_status(MAVLINK_COMM_1);
					this_seq_num.raw_imu = chan_status->current_rx_seq;
					break;
				}

				case MAVLINK_MSG_ID_COMMAND_ACK:
				{
					// printf("MAVLINK_MSG_ID_RAW_IMU\n");

					mavlink_command_ack_t packet;

					mavlink_msg_command_ack_decode(&message, &packet);
					current_messages.time_stamps.command_ack = get_time_usec();
					this_timestamps.command_ack = current_messages.time_stamps.command_ack;

					// Decode packet and set callback
					if(packet.command == MAV_CMD_DO_MOUNT_CONFIGURE)
					{
						current_messages.result_cmd_ack_msg_configure = packet.result;
					}
					else if(packet.command == MAV_CMD_DO_MOUNT_CONTROL)
					{
						current_messages.result_cmd_ack_msg_control = packet.result;
					}

					mavlink_status_t    *chan_status = mavlink_get_channel_status(MAVLINK_COMM_1);
					this_seq_num.command_ack = chan_status->current_rx_seq;

					break;
				}
				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}

			} // end: switch msgid

		} // end: if read message

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Gimbal_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

/**
 * @brief  This function shall reboot the gimbal
 * @param: NONE
 * @ret: None
 */
void
Gimbal_Interface::
set_gimbal_reboot(void)
{
	// Prepare command for off-board mode
	mavlink_command_long_t comm = { 0 };

	comm.target_system    	= system_id;
	comm.target_component 	= gimbal_id;

	comm.command            = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;

	comm.param1             = 0;
    comm.param2             = 0;
	comm.param3             = 0;
    comm.param4             = 1;
	comm.param5             = 0;
    comm.param6             = 0;
    comm.param7             = 0;
    comm.confirmation     	= true;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &comm);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send GIMBAL REBOOT \n");
	else
		// printf("%lu GIMBAL_REBOOT = [ %d] \n", write_count);

	return;
}

/**
 * @brief  This function shall turn on/off motor driver
 * @param: type see control_gimbal_motor_t
 * @ret: None
 */
void
Gimbal_Interface::
set_gimbal_motor_mode(control_gimbal_motor_t type)
{
	// Prepare command for off-board mode
	mavlink_command_long_t comm = { 0 };


	comm.target_system    	= system_id;
	comm.target_component 	= gimbal_id;

	comm.command            = MAV_CMD_USER_1;
    comm.param7             = type;	// type 0 =>off , 1=>on
    comm.confirmation     	= true;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &comm);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send MOTOR MODE \n");
	else
		// printf("%lu MOTOR_MODE  = [ %d] \n", write_count, type);

	return;
}

/**
 * @brief  This function shall turn on/off motor driver
 * @param: type see control_gimbal_motor_t
 * @ret: None
 */
void
Gimbal_Interface::
set_gimbal_mode(control_gimbal_mode_t mode)
{
	// Prepare command for off-board mode
	mavlink_command_long_t comm = { 0 };

	comm.target_system    	= system_id;
	comm.target_component 	= gimbal_id;

	comm.command            = MAV_CMD_USER_2;
    comm.param7             = mode;
    comm.confirmation     	= false;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &comm);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send GIMBAL MODE \n");
	else
		// printf("%lu GIMBAL_MODE  = [ %d] \n", write_count, mode);

	return;
}


/**
 * @brief  This function shall turn on/off motor driver
 * @param: type see control_gimbal_motor_t
 * @ret: None
 */
void
Gimbal_Interface::
set_gimbal_axes_mode(control_gimbal_axis_mode_t tilt,
                        control_gimbal_axis_mode_t roll,
                        control_gimbal_axis_mode_t pan)
{
	// Prepare command for off-board mode
	mavlink_command_long_t comm = { 0 };


	comm.target_system    	= system_id;
	comm.target_component 	= gimbal_id;
    comm.confirmation     	= false;

	/*Default all axes that stabilize mode */
    roll.stabilize  = 1;
    tilt.stabilize  = 1;
    pan.stabilize   = 1;

	comm.command            = MAV_CMD_DO_MOUNT_CONFIGURE;

    comm.param1             = MAV_MOUNT_MODE_MAVLINK_TARGETING;
    comm.param2             = roll.stabilize;
    comm.param3             = tilt.stabilize;
    comm.param4             = pan.stabilize;
    comm.param5             = roll.input_mode;
    comm.param6             = tilt.input_mode;
    comm.param7             = pan.input_mode;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &comm);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send GIMBAL AXES MODE \n");
	else
		// printf("%lu GIMBAL_AXES_MODE \n", write_count);

	return;
}


/**
 * @brief  This function shall turn on/off motor driver
 * @param: type see control_gimbal_motor_t
 * @ret: None
 */
void
Gimbal_Interface::
set_gimbal_move(int16_t tilt, int16_t roll, int16_t pan)
{
	// Prepare command for off-board mode
	mavlink_command_long_t comm = { 0 };


	comm.target_system    	= system_id;
	comm.target_component 	= gimbal_id;

	comm.command            = MAV_CMD_DO_MOUNT_CONTROL;
    comm.confirmation     	= true;

    comm.param1             = (float) (tilt);
    comm.param2             = (float) roll;
    comm.param3             = (float) (-pan);
    comm.param7             = (float) MAV_MOUNT_MODE_MAVLINK_TARGETING;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &comm);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send GIMBAL AXES MODE \n");
	else
		// printf("%lu GIMBAL_AXES_MODE \n", write_count);

	return;
}

/**
 * @brief  This function get gimbal status
 * @param: None
 * @ret: Gimbal status
 */
gimbal_status_t 
Gimbal_Interface::
get_gimbal_status(void)
{
	/* Check gimbal status has changed*/
	if(current_messages.time_stamps.sys_status)
	{
		// Get gimbal status 
		uint16_t errors_count1 = current_messages.sys_status.errors_count1;
		uint16_t errors_count2 = current_messages.sys_status.errors_count2;

		/* Check gimbal's motor */
        if(errors_count1 & STATUS1_MOTORS)
        {
            this->gimbal_status.state = GIMBAL_STATE_ON;
            
            /* Check gimbal is follow mode*/
            if(errors_count1 & STATUS1_MODE_FOLLOW_LOCK)
            {
                this->gimbal_status.mode = GIMBAL_STATE_FOLLOW_MODE;
            }
            else
            {
                this->gimbal_status.mode = GIMBAL_STATE_LOCK_MODE;
            }
        } else if(not (errors_count1 & STATUS1_MOTORS))
        {
        	this->gimbal_status.state = GIMBAL_STATE_OFF;
        	this->gimbal_status.mode  = GIMBAL_STATE_OFF;
        }
        /* Check gimbal is initializing*/
        else if(errors_count1 & STATUS1_INIT_MOTOR)
        {
            this->gimbal_status.state = GIMBAL_STATE_INIT;
        }
        else if((errors_count1 & STATUS1_SENSOR_ERROR) ||
        		(errors_count1 & STATUS1_MOTOR_PHASE_ERROR) ||
        		(errors_count1 & STATUS1_MOTOR_ANGLE_ERROR))
        {
            /* Check gimbal is error state*/
            this->gimbal_status.state = GIMBAL_STATE_ERROR;
        }
        
        /* Check gimbal's sensor status */
        if(errors_count2 & 0x01)
        {
            this->gimbal_status.sensor |= SENSOR_IMU_ERROR;
        }
        if(errors_count2 & 0x02)
        {
            this->gimbal_status.sensor |= SENSOR_EN_TILT;
        }
        if(errors_count2 & 0x04)
        {
            this->gimbal_status.sensor |= SENSOR_EN_ROLL;
        }
        if(errors_count2 & 0x08)
        {
            this->gimbal_status.sensor |= SENSOR_EN_PAN;
        }
        else 
        {
            this->gimbal_status.sensor = SENSOR_OK;
        }
	}

	return gimbal_status;
}


/**
 * @brief  This function get gimbal imu
 * @param: None
 * @ret: Gimbal status
 */
mavlink_raw_imu_t 
Gimbal_Interface::
get_gimbal_raw_imu(void)
{
	/* Check gimbal status has changed*/
	if(current_messages.time_stamps.raw_imu)
	{
		return current_messages.raw_imu;
	}
}

/**
 * @brief  This function get gimbal attitude
 * @param: None
 * @ret: Gimbal status
 */
mavlink_mount_orientation_t 
Gimbal_Interface::
get_gimbal_mount_orientation(void)
{
	/* Check gimbal status has changed*/
	if(current_messages.time_stamps.mount_orientation)
	{
		return current_messages.mount_orientation;
	}
}

/**
 * @brief  This function get gimbal encoder values
 * @param: None
 * @ret: mavlink_mount_status_t (a, b, c: pitch, roll, yaw)
 */
mavlink_mount_status_t 
Gimbal_Interface::
get_gimbal_mount_status(void)
{
	/* Check gimbal status has changed*/
	if(current_messages.time_stamps.mount_status)
	{
		return current_messages.mount_status;
	}
}


/**
 * @brief  This function get gimbal attitude 
 * @param: None
 * @ret: Gimbal status
 */
Time_Stamps 
Gimbal_Interface::
get_gimbal_time_stamps(void)
{

	return current_messages.time_stamps;
}

/**
 * @brief  This function get gimbal the sequence number of last packet received
 * @param: None
 * @ret: Sequence_Numbers
 */
Sequence_Numbers 
Gimbal_Interface::
get_gimbal_seq_num(void)
{
	return current_messages.current_seq_rx;
}

/**
 * @brief  This function get gimbal the command ack of MAV_CMD_DO_MOUNT_CONFIGURE 
 * @param: None
 * @ret: Result of command
 */
uint8_t 
Gimbal_Interface::
get_command_ack_do_mount_configure(void)
{
	/* Check gimbal command ack has changed*/
	if(current_messages.time_stamps.command_ack)
	{
		return current_messages.result_cmd_ack_msg_configure;
	}
}

/**
 * @brief  This function get gimbal the command ack of MAV_CMD_DO_MOUNT_CONTROL
 * @param: None
 * @ret: Result of command
 */
uint8_t 
Gimbal_Interface::
get_command_ack_do_mount_control(void)
{
	/* Check gimbal command ack has changed*/
	if(current_messages.time_stamps.command_ack)
	{
		return current_messages.result_cmd_ack_msg_control;
	}
}
// ------------------------------------------------------------------------------
//   Write heartbeat Message
// ------------------------------------------------------------------------
void 
Gimbal_Interface::
write_heartbeat(void)
{
	mavlink_heartbeat_t heartbeat;

	heartbeat.type 			= MAV_TYPE_ONBOARD_CONTROLLER;
	heartbeat.autopilot 	= MAV_AUTOPILOT_GENERIC;
	heartbeat.base_mode 	= 0;
	heartbeat.custom_mode 	= 0;
	heartbeat.system_status = MAV_STATE_ACTIVE;

	
	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_heartbeat_encode(SYSID_ONBOARD, MAV_COMP_ID_SYSTEM_CONTROL, &message, &heartbeat);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
		else
			// printf("%lu Write Heartbeat  \n", write_count);

	return;

}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Gimbal_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_gimbal_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------


	while  (not get_connection())
	{
		if ( time_to_exit )
		{
			printf("CHECK FOR MESSAGES sysid: %d compid: %d\n", current_messages.sysid, current_messages.compid);

			return;
		}
		usleep(500000); // Check at 2Hz
	}

	printf("Found \n");

	// We know the gimbal is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the Gimbal we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT GIMBAL SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not gimbal_id )
	{
		gimbal_id = current_messages.compid;
		printf("GOT GIMBAL COMPONENT ID: %i\n", gimbal_id);
		printf("\n");
	}
	
	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_gimbal_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Gimbal_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Gimbal_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Gimbal_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Gimbal_Interface::
handle_quit( int sig )
{

	// Send command disable 
	// disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop gimbal interface\n");
	}

}

bool Gimbal_Interface::
get_flag_exit(void)
{
	return time_to_exit;
}

bool Gimbal_Interface::
get_connection(void)
{
	return has_detected;
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Gimbal_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
		// usleep(10); //! @note CPU optimization, reduce the CPU usage a lot
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Gimbal_Interface::
write_thread(void)
{

	// Blocking wait for new data
	while ( !writing_status and !time_to_exit )
	{
		// signal startup
		writing_status = true;

		// write a message and signal writing
		write_heartbeat();

		// signal end
		writing_status = false;

		usleep(1000000); // Read batches at 10Hz
	}

	return;

}

// End Gimbal_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_gimbal_interface_read_thread(void *args)
{
	// takes an gimbal object argument
	Gimbal_Interface *gimbal_interface = (Gimbal_Interface *)args;

	// run the object's read thread
	gimbal_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_gimbal_interface_write_thread(void *args)
{
	// takes an gimbal object argument
	Gimbal_Interface *gimbal_interface = (Gimbal_Interface *)args;

	// run the object's read thread
	gimbal_interface->start_write_thread();

	// done!
	return NULL;
}

/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.*****END OF FILE**********/