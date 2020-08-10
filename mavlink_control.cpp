/*******************************************************************************
 * @file    mavlink_control.cpp
 * @author  The GremsyCo
 * @version V2.3.0
 * @date    August-21-2018
 * @brief   This file contains a example for showing how to control gimbal in
 *          some cases
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
#include "mavlink_control.h"


/* Private define-------------------------------------------------------------*/
/* Private Typedef------------------------------------------------------------*/


typedef enum _sdk_process_state
{
    STATE_IDLE,

    STATE_CHECK_FIRMWARE_VERSION,
    STATE_SETTING_GIMBAL,
    STATE_SETTING_MESSAGE_RATE,

    STATE_SET_GIMBAL_OFF,
    STATE_SET_GIMBAL_ON,
    
    STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE,
    STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW,
    STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CCW,
    
    STATE_SET_CTRL_GIMBAL_SPEED_MODE,
    STATE_MOVE_SPEED_MODE,
    
    STATE_MOVE_TO_ZERO,

    STATE_SET_GIMBAL_REBOOT,

    STATE_DONE
}sdk_process_state_t; 


typedef struct 
{
    sdk_process_state_t state;
    uint64_t            last_time_send;
    uint64_t            timeout;
} sdk_process_t;

/* Private variable- ---------------------------------------------------------*/
static sdk_process_t sdk;

// ------------------------------------------------------------------------------
//   Gimbal sample control and get data 
// ------------------------------------------------------------------------------
int
gGimbal_sample (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 115200;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);


	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);


	/*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a heartbeat 1hz It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
	Gimbal_Interface gimbal_interface(&serial_port);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	serial_port_quit        = &serial_port;
	gimbal_interface_quit 	= &gimbal_interface;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and Gimbal_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	serial_port.start();
	gimbal_interface.start();

	/// Process data 
	while (!gimbal_interface.get_flag_exit())
	{
		uint32_t time_display = (uint32_t) (get_time_usec()/1000);

		if(time_display%500 == 0 && gimbal_interface.present())
		{

            // Reset time 
            sdk.timeout = get_time_usec();

             // Sample control
			gGimbal_control_sample(gimbal_interface);

            // Sample display value
			gGimbal_displays(gimbal_interface);
		}
        else
        {
            if(get_time_usec() - sdk.timeout > 2000000)
            {
                sdk.state = STATE_IDLE;
            }
        }
	}

	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	gimbal_interface.stop();
	serial_port.stop();

	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;
}

// --------------------------------------------------------------------------
//   Paser gimbal info
// --------------------------------------------------------------------------


void gGimbal_displays(Gimbal_Interface &api)
{
	/*--------------------------------------------------------------------------
	  GET A MESSAGE
	--------------------------------------------------------------------------*/
	printf("READ SOME MESSAGES \n");
	printf("\n");

    gimbal_status_t gimbal_status = api.get_gimbal_status();
    printf("Got message gimbal status \n");

    if(gimbal_status.state == GIMBAL_STATE_OFF)
    {
        printf("Gimbal's status is OFF!\n");
    }
    else if(gimbal_status.state == GIMBAL_STATE_ON)
    {
          printf("Gimbal is operating\n");
    }
    else if(gimbal_status.state == GIMBAL_STATE_INIT)
    {
        printf("Gimbal is busy!\n");
    }
    else if(gimbal_status.state == GIMBAL_STATE_ERROR)
    {
        printf("Gimbal's status is error!\n");
    }

    mavlink_raw_imu_t imu = api.get_gimbal_raw_imu();
    imu.time_usec = api.get_gimbal_time_stamps().raw_imu;

	printf("Got message RAW IMU.\n");
	printf("\traw imu: time: %lu, xacc:%d, yacc:%d, zacc:%d, xgyro:%d, xgyro:%d, xgyro:%d(raw)\n", 
                                                    (unsigned long)imu.time_usec, 
                                                    imu.xacc, 
                                                    imu.yacc, 
                                                    imu.zacc,
                                                    imu.xgyro,
                                                    imu.ygyro,
                                                    imu.zgyro);

	mavlink_mount_orientation_t mnt_orien = api.get_gimbal_mount_orientation();
    mnt_orien.time_boot_ms = api.get_gimbal_time_stamps().mount_orientation;

	printf("Got message Mount orientation.\n");
	printf("\torientation: time: %lu, p:%f, r:%f, y:%f (degree)\n",   (unsigned long)mnt_orien.time_boot_ms, 
                                                                        mnt_orien.pitch, 
                                                                        mnt_orien.roll, 
                                                                        mnt_orien.yaw);

    mavlink_mount_status_t mnt_status = api.get_gimbal_mount_status();
    uint64_t mnt_status_time_stamp = api.get_gimbal_time_stamps().mount_status;

	printf("Got message Mount status \n");

    if(api.get_gimbal_config_mavlink_msg().enc_type_send)
    {
        printf("\tEncoder Count: time: %lu, p:%d, r:%d, y:%d (Resolution 2^16)\n", (unsigned long)mnt_status_time_stamp, 
                                                            mnt_status.pointing_a, 
                                                            mnt_status.pointing_b, 
                                                            mnt_status.pointing_c);
    }
    else
    {
        printf("\tEncoder Angle: time: %lu, p:%d, r:%d, y:%d (Degree)\n", (unsigned long)mnt_status_time_stamp, 
                                                            mnt_status.pointing_a, 
                                                            mnt_status.pointing_b, 
                                                            mnt_status.pointing_c);
    }



    gimbal_config_axis_t setting = api.get_gimbal_config_tilt_axis();

    printf("\tSETTING TILT: dir %d, speed_follow: %d, speed_control: %d\n", 
                                                            setting.dir,
                                                            setting.speed_follow,
                                                            setting.speed_control);

    gimbal_motor_control_t tilt;
    gimbal_motor_control_t roll;
    gimbal_motor_control_t pan;

    uint8_t output_filter, gyro_filter, gain;

    api.get_gimbal_motor_control(tilt, roll, pan, gyro_filter, output_filter, gain);
    printf("\tMOTOR_CONTROL: GYRO: %d, OUT %d, GAIN %d\n", gyro_filter, output_filter, gain);
    printf("\tTILT  stiff %d, hold: %d\n" , tilt.stiffness, tilt.holdstrength);
    printf("\tROLL  stiff %d, hold: %d\n" , roll.stiffness, roll.holdstrength);
    printf("\tPAN   stiff %d, hold: %d\n" , pan.stiffness, pan.holdstrength);



	printf("\n");
}

// ------------------------------------------------------------------------------
//   This example will demonstrate how to set gimbal mode 
//      and control gimbal in angle and speed mode
// ------------------------------------------------------------------------------

void gGimbal_control_sample(Gimbal_Interface &onboard)
{
    switch(sdk.state)
    {
        case STATE_IDLE:
        {
           sdk.state = STATE_CHECK_FIRMWARE_VERSION;

           sdk.last_time_send = get_time_usec();
        }
        break;
        case STATE_CHECK_FIRMWARE_VERSION:
        {

           fw_version_t fw = onboard.get_gimbal_version();
           printf("FW Version: %d.%d.%d.%s\n", fw.x, fw.y, fw.z, fw.type);


           // This firmware only apply for the firmware version from v7.x.x or above
           if(fw.x >= 7 && fw.y >= 5)
           {
                sdk.state = STATE_SETTING_GIMBAL;
           }
           else
           {
                printf("DO NOT SUPPORT FUNCTIONS. Please check the firmware version\n");
                printf("1. MOTOR CONTROL\n");
                printf("2. AXIS CONFIGURATION\n");
                printf("3. MAVLINK MSG RATE CONFIGURATION\n");
           }

           usleep(100000);
        }
        break;
        case STATE_SETTING_GIMBAL:
        {

            // Setting axis for control. see the struct gimbal_config_axis_t
            gimbal_config_axis_t config = {0};

            config = {DIR_CCW, 50, 50, 65, 50, 0};
            onboard.set_gimbal_config_tilt_axis(config);

            config = {DIR_CW, 50, 60, 0, 0, 0};
            onboard.set_gimbal_config_roll_axis(config);

            config = {DIR_CW, 50, 70, 87, 50, 0};
            onboard.set_gimbal_config_pan_axis(config);


            // Motor control likes: Stiffness, holdstrength, gyro filter, output filter and gain
            gimbal_motor_control_t tilt = {80, 40};
            gimbal_motor_control_t roll = {90, 40};
            gimbal_motor_control_t pan = {100, 40};
            onboard.set_gimbal_motor_control(tilt, roll, pan, 2, 3, 120);

            usleep(100000);

           sdk.state = STATE_SETTING_MESSAGE_RATE;
        }
        break;
        case STATE_SETTING_MESSAGE_RATE:
        {
            uint8_t emit_heatbeat = 1;
            uint8_t status_rate = 10;
            uint8_t enc_value_rate = 10; 
            uint8_t enc_type_send = 0;  // Set type of encoder is angle
            uint8_t orien_rate = 50;
            uint8_t imu_rate = 10;
            
            printf("Set msg rate!\n");

            // configuration message. Note emit_heartbeat need to emit when using this gSDK. If not, the gSDK will waiting forever.
            onboard.set_gimbal_config_mavlink_msg(  emit_heatbeat, 
                                                    status_rate, 
                                                    enc_value_rate, 
                                                    enc_type_send, 
                                                    orien_rate,
                                                    imu_rate);
            usleep(100000);

            sdk.state = STATE_SET_GIMBAL_OFF;
        }
        break;
        case STATE_SET_GIMBAL_OFF:
        {
           // Check gimbal is on
            if(onboard.get_gimbal_status().state == GIMBAL_STATE_ON)
            {
                // Turn off
                onboard.set_gimbal_motor_mode(TURN_OFF);

                printf("TURN_OFF! %d \n", onboard.get_gimbal_status().mode);
                
                sdk.last_time_send = get_time_usec();
            }
            else if(onboard.get_gimbal_status().state == GIMBAL_STATE_OFF)
            {
                if((get_time_usec() - sdk.last_time_send) > 1000000)
                {
                    sdk.last_time_send = get_time_usec();
                    
                    sdk.state = STATE_SET_GIMBAL_ON;
                }
            }
        }
        break;
        case STATE_SET_GIMBAL_ON:
        {
            // Check gimbal is on
            if(onboard.get_gimbal_status().mode == GIMBAL_STATE_OFF)
            {
                // Turn on gimbal
                onboard.set_gimbal_motor_mode(TURN_ON);
                
                printf("TURN_ON!\n");
                
                 sdk.last_time_send = get_time_usec();
            }
            else if(onboard.get_gimbal_status().mode)
            {
                if((get_time_usec() - sdk.last_time_send) > 1000000)
                {
                    sdk.last_time_send = get_time_usec();
                    
                    sdk.state = STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE;
                }
            }
        }
        break;
        case STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE:
        {
            printf("Set gimbal's yaw follow mode! %d\n", onboard.get_command_ack_do_mount_configure());
            
            control_gimbal_axis_mode_t pitch, roll, yaw;
            
            // Set gimbal axes mode.
            // NOTE: ROLL only allow controlling in ABSOLUTE_FRAME and ANGULAR_RATE.

            pitch.input_mode    = CTRL_ANGLE_ABSOLUTE_FRAME;
            roll.input_mode     = CTRL_ANGLE_ABSOLUTE_FRAME;
            yaw.input_mode      = CTRL_ANGLE_BODY_FRAME;
            
            // Set gimbal axes mode
            onboard.set_gimbal_axes_mode(pitch, roll, yaw);
            
            // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONFIGURE. 
            if(onboard.get_command_ack_do_mount_configure() == MAV_RESULT_ACCEPTED)
            {
                //Wait a moment about 5 seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 5000000)
                {
                    sdk.last_time_send = get_time_usec();
                    
                    sdk.state = STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW;
                }
            }
        }
        break;
        case STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW:
        {
            printf("Control gimbal's yaw cw follow mode! %d\n", onboard.get_command_ack_do_mount_control());
            
            // Set gimbal move to 
            float setpoint_pitch  = 40.0;
            float setpoint_roll   = 0;
            float setpoint_yaw    = 170.0;
           
            /// Set command gimbal move
            onboard.set_gimbal_move(setpoint_pitch, setpoint_roll, setpoint_yaw);

            // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONTROL
            if(onboard.get_command_ack_do_mount_control() == MAV_RESULT_ACCEPTED)
            {
                //Wait a moment about 5 seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 5000000)
                {
                    // Reset time for the next step
                    sdk.last_time_send = get_time_usec();
                    
                    // Switch to move gimbal in CCW
                    sdk.state = STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CCW;
                }
            }
        }
        break;

        case STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CCW:
        {
            printf("Control gimbal's yaw ccw follow mode! %d\n", onboard.get_command_ack_do_mount_control());
            
            // Set gimbal move to 
            float setpoint_pitch  = -40.0;
            float setpoint_roll   = 0;
            float setpoint_yaw    = -170.0;

            /// Set command gimbal move
            onboard.set_gimbal_move(setpoint_pitch, setpoint_roll, setpoint_yaw);

            // Check gimbal feedback COMMAND_ACK after sending angle
            if(onboard.get_command_ack_do_mount_control() == MAV_RESULT_ACCEPTED)
            {
                //Wait a moment about 5 seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 5000000)
                {
                    sdk.last_time_send = get_time_usec();
                    
                    sdk.state = STATE_SET_CTRL_GIMBAL_SPEED_MODE;
                }
            }
        }
        break;
        case STATE_SET_CTRL_GIMBAL_SPEED_MODE:
        {
            printf("Set move gimbal in speed mode!\n");
            
            control_gimbal_axis_mode_t pitch, roll, yaw;
            
            pitch.input_mode    = CTRL_ANGULAR_RATE;
            roll.input_mode     = CTRL_ANGULAR_RATE;
            yaw.input_mode      = CTRL_ANGULAR_RATE;
            
            onboard.set_gimbal_axes_mode(pitch, roll, yaw);
            
            // Check gimbal feedback COMMAND_ACK after sending angle
            if(onboard.get_command_ack_do_mount_configure() == MAV_RESULT_ACCEPTED)
            {
                sdk.last_time_send = get_time_usec();
            
                sdk.state = STATE_MOVE_SPEED_MODE;
            }
        }
        break;
        case STATE_MOVE_SPEED_MODE:
        {
            printf("Speed control gimbal in speed mode: \n");

            // Moving gimbal in speed mode with speed = 10 degree/second
            float setpoint_pitch = 0.1;
            float setpoint_roll  = 0;
            float setpoint_yaw   = 0.1;

            /*Get gimbal angle to check when moving in speed mode. */
            float current_pitch_angle = onboard.get_gimbal_mount_orientation().pitch;
            float current_roll_angle = onboard.get_gimbal_mount_orientation().roll;

            //  [deg] Yaw relative to vehicle
            float current_yaw_angle = onboard.get_gimbal_mount_orientation().yaw;

            printf("current_pitch_angle: %2.3f, current_roll_angle: %2.3f, current_yaw_angle: %2.3f\n", 
                                    current_pitch_angle, current_roll_angle, current_yaw_angle);

            /// Move gimbal in speed mode
            onboard.set_gimbal_move(setpoint_pitch, setpoint_roll, setpoint_yaw);


            //Moving gimbal in speed mode about 5 seconds
            if((get_time_usec() - sdk.last_time_send) > 5000000)
            {
                sdk.last_time_send = get_time_usec();
                
                sdk.state = STATE_MOVE_TO_ZERO;
            }
        }
        break;
        case STATE_MOVE_TO_ZERO:
        {
            printf("Set move angle for axes mode!\n");
            
            control_gimbal_axis_mode_t pitch, roll, yaw;
            
            // Set axes mode. 
            pitch.input_mode    = CTRL_ANGLE_ABSOLUTE_FRAME;
            roll.input_mode     = CTRL_ANGLE_ABSOLUTE_FRAME;
            yaw.input_mode      = CTRL_ANGLE_BODY_FRAME;
            
            onboard.set_gimbal_axes_mode(pitch, roll, yaw);
            
            // Check gimbal feedback COMMAND_ACK after sending angle
            if(onboard.get_command_ack_do_mount_configure() != MAV_RESULT_ACCEPTED)
            {
                break;
            }

            /* Reset time */                
            printf("move Gimbal to zero pos!\n");

            float setpoint_pitch  = 0;
            float setpoint_roll   = 0;
            float setpoint_yaw    = 0;

            /// Apply value
            onboard.set_gimbal_move(setpoint_pitch, setpoint_roll, setpoint_yaw);
            
             // Check gimbal feedback COMMAND_ACK after sending angle
            if(onboard.get_command_ack_do_mount_control() == MAV_RESULT_ACCEPTED)
            {
                //Wait a moment about 5 seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 5000000)
                {
                    sdk.last_time_send = get_time_usec();
                    
                    sdk.state = STATE_SET_GIMBAL_REBOOT;
                }
            }
        }
        break;
        case STATE_SET_GIMBAL_REBOOT:
        {
            printf("STATE_SET_GIMBAL_REBOOT!\n");
            onboard.set_gimbal_reboot();

            if((get_time_usec() - sdk.last_time_send) > 1000000)
            {
                sdk.last_time_send = get_time_usec();

                sdk.state = STATE_IDLE;
            }
        }
        break;
    }
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{
	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}
	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		gimbal_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = gGimbal_sample(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


/************************ (C) COPYRIGHT Gremsy *****END OF FILE****************/
