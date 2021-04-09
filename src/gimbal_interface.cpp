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

uint64_t
get_time_msec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000 + _time_stamp.tv_usec;
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
	current_messages.sys_status.errors_count2 = 0;
	current_messages.sys_status.errors_count1 = 0;

	_state = GIMBAL_STATE_NOT_PRESENT;

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
	bool success;           // receive success flag
	Time_Stamps 			this_timestamps;
	Sequence_Numbers 		this_seq_num;
	bool 					received_all = false;  // receive only one message

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
					// printf("MAVLINK_MSG_ID_HEARTBEAT\n");
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

					// Get time
					_last_report_msg_us = get_time_usec();

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
					// printf("MAVLINK_MSG_ID_COMMAND_ACK\n");
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
				case MAVLINK_MSG_ID_PARAM_VALUE:
				{
					// printf("MAVLINK_MSG_ID_PARAM_VALUE\n");
					mavlink_param_value_t packet;

					mavlink_msg_param_value_decode(&message, &packet);
					
					for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
					{
						// Compare the index from gimbal with the param list 
						if(packet.param_index == _params_list[i].gmb_idx)
						{
							_params_list[i].seen = true;

							switch(_params_list[i].state)
							{
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
		                    		if(packet.param_value == _params_list[i].value)
		                    		{
		                    			_params_list[i].state = PARAM_STATE_CONSISTENT;
		                    		}
	                    		break;
							}
						}
					}	
				}
				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}

			} // end: switch msgid

		} // end: if read message

		// Give the write thread time to use the port
		if(writing_status > false)
			usleep(100); 

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

// ------------------------------------------------------------------------------
//   GIMBAL Parameters
// ------------------------------------------------------------------------------
void 
Gimbal_Interface::
reset_params()
{
	_last_request_ms	= 0;

	for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
	{
		_params_list[i].value = 0;
		_params_list[i].state = PARAM_STATE_NOT_YET_READ;
		_params_list[i].fetch_attempts = 0;
		_params_list[i].seen = 0;
	}
}

void 
Gimbal_Interface::
fetch_params()
{
	for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
	{
		/// Check the param has been read before
		if(_params_list[i].state != PARAM_STATE_NOT_YET_READ)
		{
			// Then set the state to allow read again
			_params_list[i].state = PARAM_STATE_FETCH_AGAIN;
		}
	}
}

bool
Gimbal_Interface::
params_initialized()
{
	for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
	{
		if(_params_list[i].state == PARAM_STATE_NOT_YET_READ)
		{
			return false;
		}
	}

	return true;
}


bool
Gimbal_Interface::
params_received_all()
{
    for(uint8_t i=0; i<GIMBAL_NUM_TRACKED_PARAMS; i++) 
    {
        if(_params_list[i].state == PARAM_STATE_NOT_YET_READ || _params_list[i].state == PARAM_STATE_FETCH_AGAIN) 
        {
            return false;
        }
    }
    return true;
}

void
Gimbal_Interface::
get_param(param_index_t param, int16_t& value, int16_t val)
{
	if(!_params_list[param].seen)
	{
		value = val;
	}
	else
	{
		value = _params_list[param].value;
	}
}

void 
Gimbal_Interface::
set_param(param_index_t param, int16_t value) 
{
	if((_params_list[param].state == PARAM_STATE_CONSISTENT) && (_params_list[param].value == value))
	{
		return;
	}

	if(_params_list[param].state == PARAM_STATE_NONEXISTANT)
	{
		return;
	}

	_params_list[param].value = value;
	_params_list[param].state = PARAM_STATE_ATTEMPTING_TO_SET;

	// Prepare command for off-board mode
	mavlink_param_set_t param_set = { 0 };

	param_set.param_value		= value; 		/*<  Onboard parameter value*/
	param_set.target_system		= system_id; 	/*<  System ID*/
	param_set.target_component	= gimbal_id; 	/*<  Component ID*/

	mav_array_memcpy(param_set.param_id, get_param_name(param), sizeof(char)*16);
	param_set.param_type		= MAVLINK_TYPE_UINT16_T;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_param_set_encode(system_id, companion_id, &message, &param_set);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	_last_request_ms = get_time_msec();

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not set param: %s\n", get_param_name(param));
}

void 
Gimbal_Interface::
param_update()
{
	uint32_t tnow_ms = get_time_msec();

	/// Retry initia param retrieval
	if(!params_received_all())
	{
		for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
		{
			if(!_params_list[i].seen)
			{
				mavlink_param_request_read_t	request = {0};

				request.target_system    	= system_id;
				request.target_component 	= gimbal_id;
				request.param_index 		= _params_list[i].gmb_idx;

				mav_array_memcpy(request.param_id, get_param_name((param_index_t)i), sizeof(char)*16);

				printf("Request param read: %s \n", get_param_name((param_index_t)i));

				// --------------------------------------------------------------------------
				//   ENCODE
				// --------------------------------------------------------------------------
				mavlink_message_t message;
				mavlink_msg_param_request_read_encode(system_id, companion_id, &message, &request);

				// --------------------------------------------------------------------------
				//   WRITE
				// --------------------------------------------------------------------------

				// do the write
				int len = write_message(message);

				// check the write
				if ( len <= 0 )
				{
					fprintf(stderr,"WARNING: could not send Request list to gimbal\n");
				}

				// Send request read try again
				_params_list[i].fetch_attempts++;

				// Waing to read
				// usleep(100000);
			}
		}
	}

	// retry param set
	for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
	{
		if(_params_list[i].state == PARAM_STATE_ATTEMPTING_TO_SET 
			&& tnow_ms - _last_request_ms > _retry_period)
		{
			_last_request_ms = tnow_ms;

			// Prepare command for off-board mode
			mavlink_param_set_t param_set = { 0 };

			param_set.param_value		= _params_list[i].value; /*<  Onboard parameter value*/
			param_set.target_system		= system_id; 			/*<  System ID*/
			param_set.target_component	= gimbal_id; /*<  Component ID*/

			mav_array_memcpy(param_set.param_id, get_param_name((param_index_t)i), sizeof(char)*16);
			param_set.param_type		= MAVLINK_TYPE_UINT16_T;

			// --------------------------------------------------------------------------
			//   ENCODE
			// --------------------------------------------------------------------------

			mavlink_message_t message;
			mavlink_msg_param_set_encode(system_id, companion_id, &message, &param_set);

			// --------------------------------------------------------------------------
			//   WRITE
			// --------------------------------------------------------------------------

			// do the write
			int len = write_message(message);

			// check the write
			if ( len <= 0 )
				fprintf(stderr,"WARNING: could not Set param \n");

			if(!_params_list[i].seen)
			{
				// Send request read
				_params_list[i].fetch_attempts++;
			}
		}
	}

	// Check for nonexistent parameters
	for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
	{
		if(!_params_list[i].seen && _params_list[i].fetch_attempts > _max_fetch_attempts)
		{
			 _params_list[i].state = PARAM_STATE_NONEXISTANT;
        	printf("Gimbal parameter %s timed out\n", get_param_name((param_index_t)i));
		}
	}
}


// ------------------------------------------------------------------------------
//   Process when gimbal conntected
// ------------------------------------------------------------------------------
void 
Gimbal_Interface::
param_process(void)
{
	if(!get_connection())
	{
		_state = GIMBAL_STATE_NOT_PRESENT;
	}

	 switch(_state) 
	 {
        case GIMBAL_STATE_NOT_PRESENT:
            // gimbal was just connected or we just rebooted, transition to PRESENT_INITIALIZING
            reset_params();

            printf("GIMBAL_STATE_NOT_PRESENT\n");

            _state = GIMBAL_STATE_PRESENT_INITIALIZING;
            break;

        case GIMBAL_STATE_PRESENT_INITIALIZING:
        	param_update();

        	// // parameters done initializing,
        	if(params_initialized())
        	{
        		for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
				{
        			printf("Check [%s] %d \n", _params_list[i].gmb_id, _params_list[i].value);
        		}

        		_state = GIMBAL_STATE_PRESENT_ALIGNING;
        	}
        break;

        case GIMBAL_STATE_PRESENT_ALIGNING:
        	param_update();

			if(current_messages.sys_status.errors_count2 == 0x00)
        	{
        		printf("GIMBAL_STATE_PRESENT_RUNNING \n");
        		_state = GIMBAL_STATE_PRESENT_RUNNING;	
        	}
        	else
        	{
	        	printf("Gimbal Error with code: %d\n", current_messages.sys_status.errors_count2);
        	}

        break;

        case GIMBAL_STATE_PRESENT_RUNNING:
        	param_update();
        break;
    }
}



// ------------------------------------------------------------------------------
//   GIMBAL Command
// ------------------------------------------------------------------------------

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
set_gimbal_move(float tilt, float roll, float pan)
{
	// Prepare command for off-board mode
	mavlink_command_long_t comm = { 0 };


	comm.target_system    	= system_id;
	comm.target_component 	= gimbal_id;

	comm.command            = MAV_CMD_DO_MOUNT_CONTROL;
    comm.confirmation     	= true;

    comm.param1             = tilt;
    comm.param2             = roll;
    comm.param3             = -pan;
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
 * @brief  This function set motor controls setting
 * @param: tilt, roll, pan - stiffness and holdstrengtg, see user_manual (https://gremsy.com/gremsy-t3-manual/)
 * @param: gyro_filter - The coefficent for denoising the sensor filter
 * @param: output_filter - The coefficent for denoising the output filter
 * @param: gain - Defines how fast each axis will return to commanded position. 
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
void 
Gimbal_Interface::
set_gimbal_motor_control(	gimbal_motor_control_t tilt, 
							gimbal_motor_control_t roll,
							gimbal_motor_control_t pan, 
							uint8_t gyro_filter, uint8_t output_filter, uint8_t gain)
{
	set_param(GMB_PARAM_STIFFNESS_PITCH, (int16_t)tilt.stiffness);
	set_param(GMB_PARAM_HOLDSTRENGTH_PITCH, (int16_t)tilt.holdstrength);

	set_param(GMB_PARAM_STIFFNESS_ROLL, (int16_t)roll.stiffness);
	set_param(GMB_PARAM_HOLDSTRENGTH_ROLL, (int16_t)roll.holdstrength);

	set_param(GMB_PARAM_STIFFNESS_YAW, (int16_t)pan.stiffness);
	set_param(GMB_PARAM_HOLDSTRENGTH_YAW, (int16_t)pan.holdstrength);

	set_param(GMB_PARAM_OUTPUT_FILTER, (int16_t)output_filter);
	set_param(GMB_PARAM_GYRO_FILTER, (int16_t)gyro_filter);
	set_param(GMB_PARAM_GAIN, (int16_t)gain);	
}

/**
 * @brief  This function get motor controls setting
 * @param: tilt, roll, pan - stiffness and holdstrengtg, see user_manual (https://gremsy.com/gremsy-t3-manual/)
 * @param: gyro_filter - The coefficent for denoising the sensor filter
 * @param: output_filter - The coefficent for denoising the output filter
 * @param: gain - Defines how fast each axis will return to commanded position. 
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
void 
Gimbal_Interface::
get_gimbal_motor_control(	gimbal_motor_control_t& tilt, 
							gimbal_motor_control_t& roll,
							gimbal_motor_control_t& pan, 
							uint8_t& gyro_filter, uint8_t& output_filter, uint8_t& gain)
{
	int16_t value = 0;

	get_param(GMB_PARAM_STIFFNESS_PITCH, value);
	tilt.stiffness 		=  (uint8_t)value;
	get_param(GMB_PARAM_HOLDSTRENGTH_PITCH, value);
	tilt.holdstrength 	=  (uint8_t)value;

	get_param(GMB_PARAM_STIFFNESS_ROLL, value);
	roll.stiffness 		=  (uint8_t)value;
	get_param(GMB_PARAM_HOLDSTRENGTH_ROLL, value);
	roll.holdstrength 	=  (uint8_t)value;

	get_param(GMB_PARAM_STIFFNESS_YAW, value);
	pan.stiffness 		=  (uint8_t)value;
	get_param(GMB_PARAM_HOLDSTRENGTH_YAW, value);
	pan.holdstrength 	=  (uint8_t)value;

	get_param(GMB_PARAM_OUTPUT_FILTER, value);
	output_filter	= (uint8_t)value;
	get_param(GMB_PARAM_GYRO_FILTER, value);
	gyro_filter		= (uint8_t)value;
	get_param(GMB_PARAM_GAIN, value);	
	gain			= (uint8_t)value;
}

/**
 * @brief  This function shall configure on the tilt axis
 * 
 * @param: config - see gimbal_config_axis_t structure
 * @note: The smooth starts with a low value of 50 
 *			Slowly increase this setting until you feel an oscillation in the tilt axis, 
 *			then reduce the setting until the oscillation subsides.
 * @ret: None
 */
void 
Gimbal_Interface::
set_gimbal_config_tilt_axis(gimbal_config_axis_t config)
{
	set_param(GMB_PARAM_SMOOTH_CONTROL_PITCH, (int16_t)config.smooth_control);
	set_param(GMB_PARAM_SMOOTH_FOLLOW_PITCH, (int16_t)config.smooth_follow);
	set_param(GMB_PARAM_WINDOW_FOLLOW_PITCH,(int16_t)config.window_follow);
	set_param(GMB_PARAM_SPEED_FOLLOW_PITCH, (int16_t)config.speed_follow);
	set_param(GMB_PARAM_SPEED_CONTROL_PITCH, (int16_t)config.speed_control);

	int16_t get_dir;
	get_param(GMB_PARAM_AXIS_DIR, get_dir);

	if( config.dir == DIR_CCW)
	{
		get_dir = get_dir | 0x01;
	}
	else
	{
		get_dir &= (~0x01);
	}

	set_param(GMB_PARAM_AXIS_DIR, get_dir);
}

/**
 * @brief  This function shall return the setting on the tilt axis
 * 
 * @param: none
 * @ret: see structure gimbal_config_axis_t
 */
gimbal_config_axis_t
Gimbal_Interface::
get_gimbal_config_tilt_axis(void)
{
	gimbal_config_axis_t setting;

	int16_t ret;

	get_param(GMB_PARAM_SMOOTH_CONTROL_PITCH,ret);
	setting.smooth_control = (uint8_t)ret;

	get_param(GMB_PARAM_SMOOTH_FOLLOW_PITCH,ret);
	setting.smooth_follow = (uint8_t)ret; 

	get_param(GMB_PARAM_WINDOW_FOLLOW_PITCH,ret);
	setting.window_follow = (uint8_t)ret; 

	get_param(GMB_PARAM_SPEED_FOLLOW_PITCH, ret);
	setting.speed_follow = (uint8_t)ret;

	get_param(GMB_PARAM_SPEED_CONTROL_PITCH,ret);
	setting.speed_control = (uint8_t)ret;

	get_param(GMB_PARAM_AXIS_DIR, ret);

	if(ret & 0x01)
	{
		setting.dir = DIR_CCW;
	}
	else if(!(ret & 0x01))
	{
		setting.dir = DIR_CW;
	}

	return setting;
}


/**
 * @brief  This function shall configure for pan axis
 * 
 * @param: none
 * @ret: see structure gimbal_config_axis_t
 */
void 
Gimbal_Interface::
set_gimbal_config_pan_axis(gimbal_config_axis_t config)
{
	set_param(GMB_PARAM_SMOOTH_CONTROL_YAW, (int16_t)config.smooth_control);
	set_param(GMB_PARAM_SMOOTH_FOLLOW_YAW, (int16_t)config.smooth_follow);
	set_param(GMB_PARAM_WINDOW_FOLLOW_YAW,(int16_t)config.window_follow);
	set_param(GMB_PARAM_SPEED_FOLLOW_YAW, (int16_t)config.speed_follow);
	set_param(GMB_PARAM_SPEED_CONTROL_YAW, (int16_t)config.speed_control);

	int16_t get_dir;
	get_param(GMB_PARAM_AXIS_DIR, get_dir);

	if( config.dir == DIR_CCW)
	{
		get_dir = get_dir | 0x02;
	}
	else
	{
		get_dir &= (~0x02);
	}

	set_param(GMB_PARAM_AXIS_DIR, get_dir);
}
/**
 * @brief  This function shall return the setting on the pan axis
 * 
 * @param: none
 * @ret: see structure gimbal_config_axis_t
 */
gimbal_config_axis_t
Gimbal_Interface::
get_gimbal_config_pan_axis(void)
{
	gimbal_config_axis_t setting;

	int16_t ret;

	get_param(GMB_PARAM_SMOOTH_CONTROL_YAW, ret);
	setting.smooth_control = (uint8_t)ret; 

	get_param(GMB_PARAM_SMOOTH_FOLLOW_YAW, ret);
	setting.smooth_follow = (uint8_t)ret; 

	get_param(GMB_PARAM_WINDOW_FOLLOW_YAW, ret);
	setting.window_follow = (uint8_t)ret; 

	get_param(GMB_PARAM_SPEED_FOLLOW_YAW, ret);
	setting.speed_follow = (uint8_t)ret;

	get_param(GMB_PARAM_SPEED_CONTROL_YAW,ret);
	setting.speed_control = (uint8_t)ret;

	get_param(GMB_PARAM_AXIS_DIR, ret);

	if(ret & 0x02)
	{
		setting.dir = DIR_CCW;
	}
	else if(!(ret & 0x02))
	{
		setting.dir = DIR_CW;
	}

	return setting;
}

/**
 * @brief  This function shall configure for roll axis
 * 
 * @param: none
 * @ret: see structure gimbal_config_axis_t
 */
void 
Gimbal_Interface::
set_gimbal_config_roll_axis(gimbal_config_axis_t config)
{
	set_param(GMB_PARAM_SMOOTH_CONTROL_ROLL, (int16_t)config.smooth_control);
	set_param(GMB_PARAM_SPEED_CONTROL_ROLL, (int16_t)config.speed_control);

	int16_t get_dir;
	get_param(GMB_PARAM_AXIS_DIR, get_dir);

	if( config.dir == DIR_CCW)
	{
		get_dir = get_dir | 0x04;
	}
	else
	{
		get_dir &= (~0x04);
	}

	set_param(GMB_PARAM_AXIS_DIR, get_dir);
}

/**
 * @brief  This function shall return the setting on the roll axis
 * 
 * @param: none
 * @ret: see structure gimbal_config_axis_t
 */
gimbal_config_axis_t
Gimbal_Interface::
get_gimbal_config_roll_axis(void)
{
	gimbal_config_axis_t setting;

	int16_t ret;

	get_param(GMB_PARAM_SMOOTH_CONTROL_ROLL, ret);
	setting.smooth_control = (uint8_t)ret;

	// Roll dosen't support in follow mode
	setting.smooth_follow = 0; 
	setting.window_follow = 0; 
	setting.speed_follow = 0;

	get_param(GMB_PARAM_SPEED_CONTROL_ROLL, ret);
	setting.speed_control = (uint8_t)ret;

	get_param(GMB_PARAM_AXIS_DIR, ret);

	if(ret & 0x04)
	{
		setting.dir = DIR_CCW;
	}
	else if(!(ret & 0x04))
	{
		setting.dir = DIR_CW;
	}

	return setting;
}

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
void 
Gimbal_Interface::
set_gimbal_config_mavlink_msg(uint8_t emit_heatbeat, 
									uint8_t status_rate, 
									uint8_t enc_value_rate, 
									uint8_t enc_type_send,
									uint8_t orien_rate,
									uint8_t imu_rate)
{
	set_param(GMB_PARAM_HEATBEAT_EMIT, (int16_t)emit_heatbeat);
	set_param(GMB_PARAM_STATUS_RATE, (int16_t)status_rate);
	set_param(GMB_PARAM_ENCODER_VALUE_RATE, (int16_t)enc_value_rate);
	set_param(GMB_PARAM_ENCODER_TYPE, (int16_t)enc_type_send);
	set_param(GMB_PARAM_ORIENTATION_RATE, (int16_t)orien_rate);
	set_param(GMB_PARAM_RAW_IMU_RATE, (int16_t)imu_rate);
}

/**
 * @brief  This function get the config of mavlink message 
 * 
 * @param: None
 * @ret: config_mavlink_message_t contains setting related to the mavlink message
 */
config_mavlink_message_t 
Gimbal_Interface::
get_gimbal_config_mavlink_msg(void)
{
	config_mavlink_message_t config;

	int16_t ret;

	get_param(GMB_PARAM_HEATBEAT_EMIT, ret);
	config.emit_heatbeat	= (uint8_t)ret;

	get_param(GMB_PARAM_STATUS_RATE, ret);
	config.status_rate		= (uint8_t)ret;

	get_param(GMB_PARAM_ENCODER_VALUE_RATE, ret);
	config.enc_value_rate	= (uint8_t)ret;

	get_param(GMB_PARAM_ENCODER_TYPE, ret);
	config.enc_type_send	= (uint8_t)ret;

	get_param(GMB_PARAM_ORIENTATION_RATE, ret);
	config.orientation_rate	= (uint8_t)ret;

	get_param(GMB_PARAM_RAW_IMU_RATE, ret);
	config.imu_rate			= (uint8_t)ret;

	return config;
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
	do
	{
		if ( time_to_exit )
		{
			printf("CHECK FOR MESSAGES sysid: %d compid: %d\n", current_messages.sysid, current_messages.compid);

			return;
		}
		usleep(500000); // Check at 2Hz

	} while(not get_connection());

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
	uint32_t timeout = get_time_usec() - _last_report_msg_us;

	// Check heartbeat from gimbal
	if(!has_detected && timeout > _time_lost_connection)
	{
		printf(" Lost Connection!\n");

		// gimbal went away
		return false;
	}
	return true;
}

bool 
Gimbal_Interface::
present()
{
	uint32_t timeout = get_time_usec() - _last_report_msg_us;

	// Check time out
	if (_state != GIMBAL_STATE_NOT_PRESENT && timeout > _time_lost_connection) 
	{
	    printf(" Not Present!\n");

        // gimbal went away
        _state = GIMBAL_STATE_NOT_PRESENT;

        return false;
    }

    return (_state != GIMBAL_STATE_NOT_PRESENT) && (_state == GIMBAL_STATE_PRESENT_RUNNING);
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
		usleep(10000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------

uint32_t time_send_param, time_send_heartbeat;

void
Gimbal_Interface::
write_thread(void)
{
	// Blocking wait for new data
	while ( !writing_status and !time_to_exit )
	{

		uint32_t tnow_ms = get_time_usec();

		// signal startup
		writing_status = true;

		if(tnow_ms - time_send_heartbeat > 1000000)
		{
			// printf("HB: %d\n", (uint32_t)(tnow_ms - time_send_heartbeat));

			time_send_heartbeat = get_time_usec();
			// write a message and signal writing
			write_heartbeat();
		}
		else if(tnow_ms - time_send_param > 500000)
		{

			time_send_param = get_time_usec();

			// Process check param
			param_process();
		}

		// signal end
		writing_status = false;
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
/************************ (C) COPYRIGHT Gremsy *****END OF FILE****************/
