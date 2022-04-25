/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    mavlink_control.c
 * @author  The GremsyCoget_flag_exit
 * @version V1.0.0
 * @date    August-021-2018
 * @brief   This file contains example how to control gimbal with API
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
    
    STATE_SET_GIMBAL_FOLLOW_MODE,

    STATE_SET_GIMBAL_ROTATION_CW_1,
    STATE_SET_GIMBAL_ROTATION_CCW_1,

    STATE_SET_GIMBAL_LOCK_MODE,

    STATE_SET_GIMBAL_ROTATION_CW_2,
    STATE_SET_GIMBAL_ROTATION_CCW_2,

    STATE_SET_GIMBAL_ROTATION_SPEED_CW,
    STATE_SET_GIMBAL_ROTATION_SPEED_CCW,
    
    STATE_MOVE_TO_ZERO,
    STATE_SWITCH_TO_RC,

    STATE_MEASURE_MESSAGE_RATE,

    STATE_SET_RESET_MODE,

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
    signal(SIGINT, quit_handler);

    /*
     * Start the port and Gimbal_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port.start();
    gimbal_interface.start();

    /// Process data 
    while (!gimbal_interface.get_flag_exit())
    {
        uint64_t time_display_ms = get_time_msec();
        uint64_t last_time_send_attitude_ms = 0;

        if (gimbal_interface.present())
        {
            // Reset time 
            sdk.timeout = get_time_usec();

            // Sample control
            gGimbal_control_sample(gimbal_interface);

            // Sample display value
            // gGimbal_displays(gimbal_interface);
        }

        /* Update autopilot attitude for gimbal to reduce pan drift*/
        if (time_display_ms - last_time_send_attitude_ms > 20) // 50Hz
        {
            attitude3f_t autopilot_attitude = { 30.f, 0.f, 0.f};

            gimbal_interface.set_autopilot_attitude(autopilot_attitude);
        }

        usleep(1000);   // 1ms
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

float get_speed(limit_angle_t limit_angle, float angle, float tgt_angle)
{
    float speed;
    // check if the angles are within limit
    if (angle < limit_angle.angle_min or angle > limit_angle.angle_max)
    {
        printf("Angle is not within limit, set speed to 0!! \n");
        speed = 0;
    }
    else if (fabsf(angle-tgt_angle) > 20)
    {
        if (angle > tgt_angle){
            speed = -10;  // set to 10 deg/s if over 10 deg away from target
        }
        else {
            speed = 10;
        }
    }
    else if (fabsf(angle-tgt_angle) > 1)
    {
        if (angle > tgt_angle){
            speed = -5;  // set to 5 deg/s if over 2 deg away from target
        }
        else {
            speed = 5;
        }
    }
    else if (fabsf(angle-tgt_angle) > 0.1)
    {
        if (angle > tgt_angle){
            speed = -0.1;  // set to 0.1 deg/s (minimum) if less than 2 deg away from target
        }
        else{
            speed= 0.1;
        }
    }
    else{
        speed = 0;   // set to 0 if less than 0.1 deg away from target
    }
    return speed;
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

    imu_t imu;
    imu = api.get_gimbal_raw_imu();

    printf("Got message RAW IMU.\n");
    printf("\traw imu: xacc:%d, yacc:%d, zacc:%d, xgyro:%d, xgyro:%d, xgyro:%d(raw)\n", 
                                                    imu.accel.x, 
                                                    imu.accel.y, 
                                                    imu.accel.z,
                                                    imu.gyro.x,
                                                    imu.gyro.y,
                                                    imu.gyro.z);

    attitude3f_t mnt_orien;
    mnt_orien = api.get_gimbal_mount_orientation();

    printf("Got message Mount orientation.\n");
    printf("\torientation: pitch:%f, roll:%f, yaw:%f (degree)\n", 
                                                        mnt_orien.pitch, 
                                                        mnt_orien.roll, 
                                                        mnt_orien.yaw);

    attitude3d_t encoder;
    encoder = api.get_gimbal_encoder();

    printf("Got message Mount status \n");

    if(api.get_gimbal_config_mavlink_msg().enc_type_send)
    {
        printf("\tEncoder Count: pitch:%d, roll:%d, yaw:%d (Resolution 2^16)\n",
                                                            encoder.pitch, 
                                                            encoder.roll, 
                                                            encoder.yaw);
    }
    else
    {
        printf("\tEncoder Angle: pitch:%d, roll:%d, yaw:%d (Degree)\n",
                                                            encoder.pitch, 
                                                            encoder.roll, 
                                                            encoder.yaw);
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

    /*Get gimbal limit information*/
    limit_angle_t limit_angle_pitch;
    limit_angle_t limit_angle_roll;
    limit_angle_t limit_angle_yaw;

    api.get_limit_angle_pitch(limit_angle_pitch);
    api.get_limit_angle_roll(limit_angle_roll);
    api.get_limit_angle_yaw(limit_angle_yaw);


    printf("Limit angle [Pitch]: upper_limit %d lower_limit %d\n", limit_angle_pitch.angle_max, limit_angle_pitch.angle_min);
    printf("Limit angle [Roll]: upper_limit %d lower_limit %d\n", limit_angle_roll.angle_max, limit_angle_roll.angle_min);
    printf("Limit angle [Yaw]: upper_limit %d lower_limit %d\n", limit_angle_yaw.angle_max, limit_angle_yaw.angle_min);


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

            // // This firmware only apply for the firmware version from v7.x.x or above
            // if(fw.x >= 7)
            // {
            //     sdk.state = STATE_SETTING_GIMBAL;
            // }
            // else
            // {
            //     printf("DO NOT SUPPORT FUNCTIONS. Please check the firmware version\n");
            //     printf("1. MOTOR CONTROL\n");
            //     printf("2. AXIS CONFIGURATION\n");
            //     printf("3. MAVLINK MSG RATE CONFIGURATION\n");
            // }

            sdk.state = STATE_SETTING_GIMBAL;
        }
        break;

        case STATE_SETTING_GIMBAL:
        {

            // Setting axis for control. see the struct gimbal_config_axis_t
            gimbal_config_axis_t config = {0};

            config = {DIR_CW, 50, 10, 50, 10, 1};
            onboard.set_gimbal_config_tilt_axis(config);

            config = {DIR_CW, 50, 10, 50, 10, 1};
            onboard.set_gimbal_config_roll_axis(config);

            config = {DIR_CW, 50, 10, 50, 10, 1};
            onboard.set_gimbal_config_pan_axis(config);

            /*Delay 1ms*/
            usleep(1000);

            // Motor control likes: Stiffness, holdstrength, gyro filter, output filter and gain
            gimbal_motor_control_t tilt = {30, 30};
            gimbal_motor_control_t roll = {40, 35};
            gimbal_motor_control_t pan = {50, 40};
            onboard.set_gimbal_motor_control(tilt, roll, pan, 2, 4, 150);

            /*Delay 1ms*/
            usleep(1000);

            /*Set limit */
            limit_angle_t limit_angle_pitch = {-90, 90};
            limit_angle_t limit_angle_roll = {-45, 45};
            limit_angle_t limit_angle_yaw = {-320, 320};
            onboard.set_limit_angle_pitch(limit_angle_pitch);
            onboard.set_limit_angle_roll(limit_angle_roll);
            onboard.set_limit_angle_yaw(limit_angle_yaw);

            /*Delay 1ms*/
            usleep(1000);

            /* Set enable combine attitude from the aircraft*/
            /* Uncomment the line below to enable gimbal reduce pan drift */
            // onboard.set_gimbal_combine_attitude(false);

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

                printf("Set TURN_OFF! Current - mode %d & state: %d\n", onboard.get_gimbal_status().mode, onboard.get_gimbal_status().state);
                
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
                
                printf("Set TURN_ON! Current - mode %d & state: %d\n", onboard.get_gimbal_status().mode, onboard.get_gimbal_status().state);

                
                 sdk.last_time_send = get_time_usec();
            }
            else if(onboard.get_gimbal_status().mode)
            {
                if((get_time_usec() - sdk.last_time_send) > 1000000)
                {
                    sdk.last_time_send = get_time_usec();
                    
                    sdk.state = STATE_SET_GIMBAL_FOLLOW_MODE;
                }
            }
        }
        break;

        case STATE_SET_GIMBAL_FOLLOW_MODE:
        {
            if(onboard.set_gimbal_follow_mode_sync() == MAV_RESULT_ACCEPTED)
            {
                //Wait a moment about 5 seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 10000000)
                {
                    sdk.last_time_send = get_time_usec();
                    
                    sdk.state = STATE_SET_GIMBAL_ROTATION_CW_1;

                    printf("Set gimbal yaw FOLLOW mode accepted !\n");
                }
            } else {
                printf("Set gimbal yaw FOLLOW mode!\n");
            }
        }
        break;

        case STATE_SET_GIMBAL_ROTATION_CW_1:
        {
            /*Set Pitch up 40 degrees and Pan rotates 300 degrees clockwise */
            float setpoint_pitch  = 40.f;
            float setpoint_roll   = 0.f;
            float setpoint_yaw    = 170.f;
         
            /// Set command gimbal move
            uint8_t res = onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);

            /* Check delta to make sure gimbal has move complete. */
            float delta_pitch_angle = fabsf(onboard.get_gimbal_mount_orientation().pitch - setpoint_pitch);
            float delta_roll_angle  = fabsf(onboard.get_gimbal_mount_orientation().roll - setpoint_roll);
            float delta_yaw_angle   = fabsf(onboard.get_gimbal_mount_orientation().yaw - setpoint_yaw);


            printf("Moving clockwise gimbal RYP [%3.2f - %3.2f - %3.2f] [Result: %d]\n",  setpoint_roll,
                                                                                          setpoint_yaw,
                                                                                          setpoint_pitch,
                                                                                          res);
            bool complete_command = false;

            if(delta_pitch_angle < 1.f &&
               delta_roll_angle < 1.f  &&
               delta_yaw_angle < 1.f )
            {
                complete_command = true;
            }

            // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONTROL
            if(res == MAV_RESULT_ACCEPTED || complete_command) {
                //Wait a moment about some seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 5000000)
                {
                    // Reset time for the next step
                    sdk.last_time_send = get_time_usec();
                    
                    // Switch to move gimbal in CCW
                    sdk.state = STATE_SET_GIMBAL_ROTATION_CCW_1;
                }
            }
        }
        break;

        case STATE_SET_GIMBAL_ROTATION_CCW_1:
        {            
            /*Set Pitch down 90 degrees and Pan rotates 300 degrees counter-clockwise */
            float setpoint_pitch  = -90.f;
            float setpoint_roll   = 0;
            float setpoint_yaw    = -170.f;

            /// Set command gimbal move
            int res = onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);

            printf("Moving Counter-Clockwise gimbal RYP [%3.2f - %3.2f - %3.2f] [Result: %d]\n",setpoint_roll,
                                                                                                setpoint_yaw,
                                                                                                setpoint_pitch,
                                                                                                res);


            /* Check delta to make sure gimbal has move complete. */
            float delta_pitch_angle = fabsf(onboard.get_gimbal_mount_orientation().pitch - setpoint_pitch);
            float delta_roll_angle  = fabsf(onboard.get_gimbal_mount_orientation().roll - setpoint_roll);
            float delta_yaw_angle   = fabsf(onboard.get_gimbal_mount_orientation().yaw - setpoint_yaw);

            bool complete_command = false;

            if(delta_pitch_angle < 1.f &&
               delta_roll_angle < 1.f  &&
               delta_yaw_angle < 1.f )
            {
                complete_command = true;
            } 
            // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONTROL
            if(res == MAV_RESULT_ACCEPTED || complete_command) {
                //Wait a moment about some seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 10000000)
                {
                    sdk.last_time_send = get_time_usec();
                    
                    sdk.state = STATE_SET_GIMBAL_LOCK_MODE;
                }
            }
        }
        break;

        case STATE_SET_GIMBAL_LOCK_MODE:
        {
            if(onboard.set_gimbal_lock_mode_sync() == MAV_RESULT_ACCEPTED) 
            {
                //Wait a moment about 5 seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 10000000)
                {
                    sdk.last_time_send = get_time_usec();
                    
                    sdk.state = STATE_SET_GIMBAL_ROTATION_CW_2;

                    printf("Set gimbal yaw LOCK mode accepted !\n");
                }
            } 
            else {
                printf("Set gimbal yaw LOCK mode!\n");
            }
        }
        break;

        case STATE_SET_GIMBAL_ROTATION_CW_2:
        {
            /*Set Pitch up 40 degrees and Pan rotates 320 degrees clockwise */
            float setpoint_pitch  = 40.f;
            float setpoint_roll   = 0.f;
            float setpoint_yaw    = 170.f;
         
            /// Set command gimbal move
            int res = onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);

            printf("Moving Clockwise gimbal RYP [%3.2f - %3.2f - %3.2f] [Result: %d]\n",setpoint_roll,
                                                                                        setpoint_yaw,
                                                                                        setpoint_pitch,
                                                                                        res);

            /* Check delta to make sure gimbal has move complete. */
            float delta_pitch_angle = fabsf(onboard.get_gimbal_mount_orientation().pitch - setpoint_pitch);
            float delta_roll_angle  = fabsf(onboard.get_gimbal_mount_orientation().roll - setpoint_roll);
            float delta_yaw_angle   = fabsf(onboard.get_gimbal_mount_orientation().yaw - setpoint_yaw);
            
            bool complete_command = false;

            if(delta_pitch_angle < 1.f &&
               delta_roll_angle < 1.f  &&
               delta_yaw_angle < 1.f )
           {
                complete_command = true;
           } 
            // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONTROL
            if(res == MAV_RESULT_ACCEPTED || complete_command) {
                //Wait a moment about 5 seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 10000000)
                {
                    // Reset time for the next step
                    sdk.last_time_send = get_time_usec();
                    
                    // Switch to move gimbal in CCW
                    sdk.state = STATE_SET_GIMBAL_ROTATION_CCW_2;
                }
            }
        }
        break;

        case STATE_SET_GIMBAL_ROTATION_CCW_2:
        {            
            /*Set Pitch down 90 degrees and Pan rotates 320 degrees counter-clockwise */
            float setpoint_pitch  = -90.f;
            float setpoint_roll   = 0;
            float setpoint_yaw    = -170.f;

             /// Set command gimbal move
            int res = onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);

            printf("Moving Counter-Clockwise gimbal RYP [%3.2f - %3.2f - %3.2f] [Result: %d]\n",setpoint_roll,
                                                                                                setpoint_yaw,
                                                                                                setpoint_pitch,
                                                                                                res);
            /* Check delta to make sure gimbal has move complete. */
            float delta_pitch_angle = fabsf(onboard.get_gimbal_mount_orientation().pitch - setpoint_pitch);
            float delta_roll_angle  = fabsf(onboard.get_gimbal_mount_orientation().roll - setpoint_roll);
            float delta_yaw_angle   = fabsf(onboard.get_gimbal_mount_orientation().yaw - setpoint_yaw);
            
            bool complete_command = false;

            if(delta_pitch_angle < 1.f &&
               delta_roll_angle < 1.f  &&
               delta_yaw_angle < 1.f )
            {
                complete_command = true;
            } 
            // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONTROL
            if(res == MAV_RESULT_ACCEPTED || complete_command) {
                //Wait a moment about 5 seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 10000000)
                {
                    sdk.last_time_send = get_time_usec();
                    
                    // onboard.set_gimbal_reboot();
                     sdk.state = STATE_SET_GIMBAL_ROTATION_SPEED_CW;
                }
            }
        }
        break;

        case STATE_SET_GIMBAL_ROTATION_SPEED_CW:
        {
            // Set gimbal move to 
            float speed_pitch   = 0.f;
            float speed_roll    = 0.f;
            float speed_yaw     = 10.f;

             /// Set command gimbal move
            int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);

            printf("Moving Speed Clockwise gimbal RYP [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,
                                                                                              speed_pitch,
                                                                                              speed_yaw,
                                                                                              res);

            //Wait a moment about 5 seconds. Just see the effect
            if((get_time_usec() - sdk.last_time_send) > 10000000)
            {
                sdk.last_time_send = get_time_usec();
                
                sdk.state = STATE_SET_GIMBAL_ROTATION_SPEED_CCW;
            }
        }
        break;

        case STATE_SET_GIMBAL_ROTATION_SPEED_CCW:
        {
            // Set gimbal move to 
            float speed_pitch   = 0.f;
            float speed_roll    = 0.f;
            float speed_yaw     = -10.f;

             /// Set command gimbal move
            int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);

            printf("Moving Speed Counter-Clockwise gimbal RYP [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,
                                                                                                        speed_pitch,
                                                                                                        speed_yaw,
                                                                                                        res);

            //Wait a moment about 5 seconds. Just see the effect
            if((get_time_usec() - sdk.last_time_send) > 10000000)
            {
                sdk.last_time_send = get_time_usec();
                
                sdk.state = STATE_SET_RESET_MODE;
            }
        }
        break;

        case STATE_MOVE_TO_ZERO:
        {
            // Set gimbal move to 
            float setpoint_pitch  = 0.f;
            float setpoint_roll   = 0.f;
            float setpoint_yaw    = 0.f;
         
            /// Set command gimbal move
            int res = onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);

            /* Check delta to make sure gimbal has move complete. */
            float delta_pitch_angle = fabsf(onboard.get_gimbal_mount_orientation().pitch - setpoint_pitch);
            float delta_roll_angle  = fabsf(onboard.get_gimbal_mount_orientation().roll - setpoint_roll);
            float delta_yaw_angle   = fabsf(onboard.get_gimbal_mount_orientation().yaw - setpoint_yaw);


            printf("Moving zero gimbal RYP [%3.2f - %3.2f - %3.2f] [Result: %d]\n",delta_pitch_angle,
                                                                                        delta_roll_angle,
                                                                                        delta_yaw_angle,
                                                                                        res);

            // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONTROL
            if(res == MAV_RESULT_ACCEPTED) {
                //Wait a moment about 5 seconds. Just see the effect
                if((get_time_usec() - sdk.last_time_send) > 10000000)
                {
                    // Reset time for the next step
                    sdk.last_time_send = get_time_usec();
                    
                    // Switch to move gimbal in CCW
                    // sdk.state = STATE_SET_GIMBAL_REBOOT;
                    sdk.state = STATE_SWITCH_TO_RC;
                }
            }
        }
        break;

        case STATE_SET_RESET_MODE:
        {
            onboard.set_gimbal_reset_mode(GIMBAL_RESET_MODE_PITCH_AND_YAW);
            
            printf("Set Gimbal Reset mode yaw and Pitch\r\n");

            sdk.state = STATE_SET_GIMBAL_FOLLOW_MODE;

        }
        break;

        case STATE_SWITCH_TO_RC: 
        {
            static uint8_t res = MAV_RESULT_IN_PROGRESS;

            if(res != MAV_RESULT_ACCEPTED)
            {
                res = onboard.set_gimbal_rc_input();
            
                printf("Set to RC input %d \n", res);
            } 
            else 
            {
                // sdk.state = STATE_SET_GIMBAL_REBOOT;
            }

            printf("Gimbal attitude: [%3.2f - %3.2f - %3.2f]\n",onboard.get_gimbal_mount_orientation().pitch,
                                                                onboard.get_gimbal_mount_orientation().roll,
                                                                onboard.get_gimbal_mount_orientation().yaw);
        }
        break;

        case STATE_MEASURE_MESSAGE_RATE:
        {
            printf("\n");

            printf("| HEARTBEAT             | %2f|\n", onboard.get_heartbeat_rate());
            printf("| SYS_STATUS            | %2f|\n", onboard.get_sys_status_rate());
            printf("| MOUNT_STATUS          | %2f|\n", onboard.get_mount_status_rate());
            printf("| MOUNT_ORIENTATION     | %2f|\n", onboard.get_mount_orientation_rate());
            printf("| RAW_IMU               | %2f|\n", onboard.get_raw_imu_rate());
            printf("| COMPONENT_INFORMATION | %2f|\n", onboard.get_comp_info_rate());

            usleep(1000000);
        }
        break;

        case STATE_SET_GIMBAL_REBOOT:
        {
            // printf("STATE_SET_GIMBAL_REBOOT!\n");
            // onboard.set_gimbal_reboot();

            // if((get_time_usec() - sdk.last_time_send) > 1000000)
            // {
            //     sdk.last_time_send = get_time_usec();

            //     sdk.state = STATE_IDLE;
            // }
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


/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.*****END OF FILE**********/
