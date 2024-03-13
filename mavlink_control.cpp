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

/* Uncomment line below to use MAVLink Gimbal Protocol V1 */
// #define _USE_MAVLINK_GIMBAL_V1

/* Private Typedef------------------------------------------------------------*/

enum sdk_process_state_t {
    STATE_IDLE,
    STATE_CONNECTED,
    STATE_PROCESS,

    STATE_DONE
};

enum __two_axis_gimbal_mount_mode_t
{
    TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT = 0x00,
    TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT,
};

struct sdk_process_t {
    sdk_process_state_t state = STATE_IDLE;
    uint64_t            timeout_ms;
};

/* Private variable ----------------------------------------------------------*/
static sdk_process_t sdk;
static Serial_Port *serial_port_quit;
static Gimbal_Interface *gimbal_interface_quit;
static __two_axis_gimbal_mount_mode_t mnt_mode = TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT;

#ifdef _USE_MAVLINK_GIMBAL_V1
static Gimbal_Interface::MAVLINK_PROTO    mav_gimbal_proto = Gimbal_Interface::MAVLINK_GIMBAL_V1;
#else
static Gimbal_Interface::MAVLINK_PROTO    mav_gimbal_proto = Gimbal_Interface::MAVLINK_GIMBAL_V2;
#endif

/* Private prototype ---------------------------------------------------------*/
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);
void quit_handler(int sig);
void gGimbal_control_sample(Gimbal_Interface &onboard);
void gGimbal_displays(Gimbal_Interface &api);

static void control_sample_gimbal_setup_param_startup(Gimbal_Interface &onboard);

void control_sample_gimbal_process(Gimbal_Interface &onboard);

// ------------------------------------------------------------------------------
//   Gimbal sample control and get data
// ------------------------------------------------------------------------------
int gGimbal_sample(int argc, char **argv)
{
    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS
    // --------------------------------------------------------------------------
    // Default input arguments
    #ifdef __APPLE__
    char *uart_name = (char *)"/dev/tty.usbmodem1";
    #else
    char *uart_name = (char *)"/dev/ttyUSB0";
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
    Gimbal_Interface gimbal_interface(&serial_port, 1, MAV_COMP_ID_ONBOARD_COMPUTER, mav_gimbal_proto, MAVLINK_COMM_0);    
    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit      = &serial_port;
    gimbal_interface_quit = &gimbal_interface;
    signal(SIGINT, quit_handler);
    /*
     * Start the port and Gimbal_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port.start();
    gimbal_interface.start();

    /// Process data
    while (!gimbal_interface.get_flag_exit()) {
        
        if(sdk.state == STATE_IDLE){
            if (gimbal_interface.present()) {
                sdk.state = STATE_CONNECTED;
            }
        } else if(sdk.state == STATE_CONNECTED) {
            control_sample_gimbal_setup_param_startup(gimbal_interface);

            sdk.state = STATE_PROCESS;
        } else {
            control_sample_gimbal_process(gimbal_interface);

        }

        usleep(1000);   // Run at 1kHz
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

static void control_sample_gimbal_set_stiffness_param(Gimbal_Interface &onboard){
    const uint8_t tilt_stiffness        = 50;
    const uint8_t tilt_holdstrength     = 30;

    const uint8_t roll_stiffness        = 50;
    const uint8_t roll_holdstrength     = 30;

    const uint8_t pan_stiffness         = 50;
    const uint8_t pan_holdstrength      = 30;

    const uint8_t gyro_filter           = 5;
    const uint8_t output_filter         = 1;

    // Motor control likes: Stiffness, holdstrength, gyro filter, output filter and gain
    // Uncomment block below to configure gimbal motor
    Gimbal_Interface::gimbal_motor_control_t tilt = { tilt_stiffness, tilt_holdstrength };
    Gimbal_Interface::gimbal_motor_control_t roll = { roll_stiffness, roll_holdstrength };
    Gimbal_Interface::gimbal_motor_control_t pan = { pan_stiffness, pan_holdstrength };
    onboard.set_gimbal_motor_control( tilt, roll, pan, gyro_filter, output_filter ); 
}
static void control_sample_gimbal_set_follow_param(Gimbal_Interface &onboard){
    const int8_t tilt_dir               = Gimbal_Interface::DIR_CW;
    const uint8_t tilt_speed_control    = 50;
    const uint8_t tilt_smooth_control   = 20;
    const uint8_t tilt_smooth_follow    = 50;
    const uint8_t tilt_window_follow    = 10;

    const int8_t roll_dir               = Gimbal_Interface::DIR_CW;
    const uint8_t roll_speed_control    = 50;
    const uint8_t roll_smooth_control   = 20;
    const uint8_t roll_smooth_follow    = 50;
    const uint8_t roll_window_follow    = 10;

    const int8_t pan_dir                = Gimbal_Interface::DIR_CW;
    const uint8_t pan_speed_control     = 50;
    const uint8_t pan_smooth_control    = 20;
    const uint8_t pan_smooth_follow     = 50;
    const uint8_t pan_window_follow     = 10;

    Gimbal_Interface::gimbal_config_axis_t config = { 0 };
    config = { tilt_dir, tilt_speed_control, tilt_smooth_control, tilt_smooth_follow, tilt_window_follow };   // Tilt
    onboard.set_gimbal_config_tilt_axis(config);
    config = { roll_dir, roll_speed_control, roll_smooth_control, roll_smooth_follow, roll_window_follow };    // Roll
    onboard.set_gimbal_config_roll_axis(config);
    config = { pan_dir, pan_speed_control, pan_smooth_control, pan_smooth_follow, pan_window_follow };  // Yaw
    onboard.set_gimbal_config_pan_axis(config);
}
static void control_sample_gimbal_setup_param_startup(Gimbal_Interface &onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;

    if(mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V1){
        GSDK_DebugInfo("Use mavlink gimbal V1");
    }else{
        GSDK_DebugInfo("Use mavlink gimbal V2");
    }

    Gimbal_Interface::fw_version_t fw = onboard.get_gimbal_version();
    GSDK_DebugInfo("Gimbal Firmware version is %d.%d.%d.%s", fw.x, fw.y, fw.z, fw.type);

    if(fw.x <= 7 && fw.y <= 7 && fw.z <= 3){
        while(1){
            GSDK_DebugInfo("\n\rCurrent gSDK not support for this gimbal firmware version !!!\n\r"
            "Please contact to GREMSY support team for get correct version !!!\n\r"
            "Thank you !!!\n\r");

            usleep(1000000);
        }
    }

    const uint8_t enc_value_rate = 10;
    const uint8_t orien_rate = 10;
    const uint8_t imu_rate = 10;

    GSDK_DebugInfo("Setting gimbal mavlink message");

    GSDK_DebugWarning("Set encoder messages rate: %dHz\n", enc_value_rate);
    onboard.set_msg_encoder_rate(enc_value_rate);
    GSDK_DebugWarning("Set mount orientation messages rate: %dHz\n", orien_rate);
    onboard.set_msg_mnt_orient_rate(orien_rate);
    GSDK_DebugWarning("Set gimbal device attitude status messages rate: %dHz\n", orien_rate);
    onboard.set_msg_attitude_status_rate(orien_rate);
    GSDK_DebugWarning("Set raw imu messgaes rate: %dHz\n", imu_rate);
    onboard.set_msg_raw_imu_rate(imu_rate);
    GSDK_DebugWarning("Set gimbal send raw encoder value.\n");
    onboard.set_gimbal_encoder_type_send(true);
    GSDK_DebugWarning("Request gimbal device information.\n");
    onboard.request_gimbal_device_info();

    char number = 0;
    printf("\t\n\r Please Enter y/n(yes or no) to setting gimbal stiffness - follow param\n\r");
    scanf("%c", &number);

    if(number == 'Y'|| number == 'y')
    {
        GSDK_DebugInfo("Setting gimbal stiffness - follow parametter");

        control_sample_gimbal_set_stiffness_param(onboard);

        control_sample_gimbal_set_follow_param(onboard);

    }

    /// Please Uncomment to enable return home when change gimbal mode
    // do{
    //     /* code */
    //     res = onboard.set_gimbal_return_home_when_change_mode(true);
    // } while (res != Gimbal_Protocol::SUCCESS);

    printf("\t\n\r Please Enter number to select two axis gimbal mount mode \n\r");
    printf("\t 0. Roll-Tilt Mount\n\r");
    printf("\t 1. Pan-Tilt Mount\n\r");
    scanf("%d", &mnt_mode);
    GSDK_DebugInfo("You selected gimbal mode %s", (mnt_mode == 0) ? ("Roll-Tilt Mount") : ("Pan-Tilt Mount"));
}
static void control_sample_gimbal_off(Gimbal_Interface &onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    static uint8_t time_out_sample = 0;

    // Check gimbal is on
    if (onboard.get_gimbal_status().state != Gimbal_Interface::GIMBAL_STATE_OFF) {
        do{
            res = onboard.set_gimbal_motor(Gimbal_Interface::TURN_OFF);
            usleep(500000);

            if(++time_out_sample >= 10){
                GSDK_DebugError("Could not TURN GIMBAL OFF!\n");
                time_out_sample = 0;
                return;
            }

        }while (res != Gimbal_Protocol::SUCCESS);

        GSDK_DebugInfo("TURN GIMBAL OFF Successfully!\n");

    } else if (onboard.get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_OFF) {
        GSDK_DebugWarning("Gimbal is OFF");
    }
}
static void control_sample_gimbal_on(Gimbal_Interface &onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    static uint8_t time_out_sample = 0;

    // Check gimbal is off
    if (onboard.get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_OFF) {
        do{
            res = onboard.set_gimbal_motor(Gimbal_Interface::TURN_ON);
            usleep(500000);

            if(++time_out_sample >= 10){
                GSDK_DebugError("Could not TURN GIMBAL ON!\n");
                time_out_sample = 0;
                return;
            }

        }while (res != Gimbal_Protocol::SUCCESS);

        GSDK_DebugInfo("TURN GIMBAL ON Successfully!\n");

    } else if (onboard.get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_FOLLOW_MODE) {
        GSDK_DebugWarning("Gimbal is ON");
    }
}
static bool control_sample_gimbal_set_lock_mode(Gimbal_Interface &onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    static uint8_t time_out_sample = 0;

    do{
        res = onboard.set_gimbal_lock_mode_sync();
        
        if(++time_out_sample >= 10){
            GSDK_DebugError("Could not set gimbal to LOCK MODE! Result code: %d\n", res);
            time_out_sample = 0;
            return false;
        }

    }while(res != Gimbal_Protocol::SUCCESS);

    GSDK_DebugInfo("Set gimbal to LOCK MODE Successfully!\n");

    return true;
}
static bool control_sample_gimbal_set_follow_mode(Gimbal_Interface &onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    static uint8_t time_out_sample = 0;

    do{
        res = onboard.set_gimbal_follow_mode_sync();

        if(++time_out_sample >= 10){
            GSDK_DebugError("Could not set gimbal to FOLLOW MODE! Result code: %d\n", res);
            time_out_sample = 0;
            return false;
        }

    }while(res != Gimbal_Protocol::SUCCESS);

    GSDK_DebugInfo("Set gimbal to FOLLOW MODE Successfully!\n");

    return true;
}
static void control_sample_gimbal_set_mapping_mode(Gimbal_Interface &onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    static uint8_t time_out_sample = 0;

    do{
        res = onboard.set_gimbal_mapping_sync();

        if(++time_out_sample >= 10){
            GSDK_DebugError("Could not set gimbal to MAPPING MODE! Result code: %d\n", res);
            time_out_sample = 0;
            return;
        }

    }while(res != Gimbal_Protocol::SUCCESS);

    GSDK_DebugInfo("Set gimbal to MAPPING MODE Successfully!\n");
}
static void control_sample_gimbal_set_move_angle(Gimbal_Interface &onboard, float pitch_angle, float roll_angle, float yaw_angle){
    // Target attitude
    float setpoint_pitch = pitch_angle;
    float setpoint_roll  = roll_angle;
    float setpoint_yaw   = yaw_angle;
    GSDK_DebugInfo("[LOCK] Move gimbal to Pitch - Roll - Yaw: %.2f - %.2f - %.2f\n", setpoint_pitch, setpoint_roll, setpoint_yaw);
    Gimbal_Protocol::result_t res = onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw);

    if (res == Gimbal_Protocol::SUCCESS) {
        GSDK_DebugInfo("\tSend command successfully!\n");
        attitude<float> attitude;

        if(mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT){
            do {
                attitude = onboard.get_gimbal_attitude();
                GSDK_DebugInfo("\tGimbal attitude Pitch - Roll - Yaw: %.2f - %.2f - %.2f\n", attitude.pitch, attitude.roll, attitude.yaw);
                onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw);
                usleep(500000);
            } while ((fabsf(attitude.pitch - setpoint_pitch) > 0.5f) ||
                    (fabsf(attitude.roll - setpoint_roll) > 0.5f));
        } else {
            do {
                attitude = onboard.get_gimbal_attitude();
                GSDK_DebugInfo("\tGimbal attitude Pitch - Roll - Yaw: %.2f - %.2f - %.2f\n", attitude.pitch, attitude.roll, attitude.yaw);
                onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw);
                usleep(500000);
            } while ((fabsf(attitude.pitch - setpoint_pitch) > 0.5f) ||
                    (fabsf(attitude.yaw - setpoint_yaw) > 0.5f));
        }

        GSDK_DebugInfo("\tGimbal attitude Pitch - Roll - Yaw: %.2f - %.2f - %.2f\n", attitude.pitch, attitude.roll, attitude.yaw);

    } else {
        GSDK_DebugError("\tCould not control gimbal! Result code: %d\n", res);
    }
}
static void control_sample_gimbal_set_move_rate(Gimbal_Interface &onboard, float pitch_rate, float roll_rate, float yaw_rate){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    GSDK_DebugInfo("\tGimbal pitch up at rate %.2fdeg/s\n", pitch_rate);

    /* Send command move */
    do {
        usleep(500000);
        res = onboard.set_gimbal_rotation_rate_sync(pitch_rate, 0.f, 0.f);
    } while (res != Gimbal_Protocol::SUCCESS);

    usleep(5000000);    // Move in 5s

    /* Stop moving */
    do {
        usleep(500000);
        res = onboard.set_gimbal_rotation_rate_sync(0.f, 0.f, 0.f);
    } while (res != Gimbal_Protocol::SUCCESS);
    
    GSDK_DebugInfo("\tGimbal yaw to East at rate %.2fdeg/s\n", yaw_rate);
    
    /* Send command move */
    do {
        usleep(500000);
        res = onboard.set_gimbal_rotation_rate_sync(0.f, 0.f, yaw_rate);
    } while (res != Gimbal_Protocol::SUCCESS);

    usleep(5000000);    // Move in 5s

    /* Stop moving */
    do {
        usleep(500000);
        res = onboard.set_gimbal_rotation_rate_sync(0.f, 0.f, 0.f);
    } while (res != Gimbal_Protocol::SUCCESS);
}
static void control_sample_gimbal_return_home(Gimbal_Interface &onboard){
    attitude<float> attitude;
    static uint8_t time_out_sample = 0;
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    
    /* Wait for returning home */
    do {
        res = onboard.set_gimbal_return_home_sync();

        usleep(500000);

        if(++time_out_sample >= 10){
            GSDK_DebugError("Could not set gimbal to RETURN HOME! Result code: %d\n", res);
            time_out_sample = 0;
            return;
        }

    } while (   res != Gimbal_Protocol::SUCCESS ||
                fabsf(attitude.pitch) > 0.5f ||
                fabsf(attitude.roll) > 0.5f ||
                fabsf(attitude.yaw) > 0.5f);

    GSDK_DebugInfo("Gimbal RETURN HOME Successfully!\n");            
}
static void control_sample_gimbal_reboot(Gimbal_Interface &onboard){
    if (onboard.set_gimbal_reboot() == Gimbal_Protocol::SUCCESS) {
        while (onboard.get_gimbal_status().state != Gimbal_Interface::GIMBAL_STATE_ON)
            usleep(2000000);
    }

    GSDK_DebugInfo("Gimbal reboot is Successfully!\n");
}

void control_sample_gimbal_process(Gimbal_Interface &onboard){

    if (onboard.present() == false) {
        return;
    }

    int number = 0;

    printf("\33[39m\n\r Please Enter number [0-10] to seclect Gimbal control mode\n\r");
    printf("\t 0. OFF Gimbal\n\r");
    printf("\t 1. ON Gimbal\n\r");
    printf("\t 2. Set Gimbal to LOCK mode\n\r");
    printf("\t 3. Set Gimbal to move angle in LOCK mode\n\r");
    printf("\t 4. Set Gimbal to move rate in LOCK mode\n\r");
    printf("\t 5. Set Gimbal to FOLLOW mode\n\r");
    printf("\t 6. Set Gimbal to move angle in FOLLOW mode\n\r");
    printf("\t 7. Set Gimbal to move rate in FOLLOW mode\n\r");
    printf("\t 8. Set Gimbal to MAPPING mode\n\r");
    printf("\t 9. Set Gimbal to Return Home\n\r");\
    printf("\t 10. Set Gimbal Reboot\n\r");

    scanf("%d", &number);

    switch (number)
    {
        case 0: // OFF Gimbal
        /* code */
        {
            control_sample_gimbal_off(onboard);
        }
        break;

        case 1: // ON Gimbal
        /* code */
        {
            control_sample_gimbal_on(onboard);
        }
        break;

        case 2: // Set Gimbal to LOCK mode
        /* code */
        {
            control_sample_gimbal_set_lock_mode(onboard);
        }
        break;

        case 3: // Set Gimbal to move angle in LOCK mode
        /* code */
        {
            /// set gimbal to LOCK mode
            if(control_sample_gimbal_set_lock_mode(onboard) == true){
                /// control gimbal moving
                // control_sample_gimbal_set_move_angle(onboard, 40.f, 0.f, 170.f);
                control_sample_gimbal_set_move_angle(onboard, 40.f, 0.f, 65.f);
            }
        }
        break;

        case 4: // Set Gimbal to move rate in LOCK mode
        /* code */
        {
            /// set gimbal to LOCK mode
            if(control_sample_gimbal_set_lock_mode(onboard) == true){
                /// control gimbal moving
                control_sample_gimbal_set_move_rate(onboard, 10.f, 0, 10.f);
            }
        }
        break;

        case 5: // Set Gimbal to FOLLOW mode
        /* code */
        {
            control_sample_gimbal_set_follow_mode(onboard);
        }
        break;

        case 6: // Set Gimbal to move angle in FOLLOW mode
        /* code */
        {
            /// set gimbal to FOLLOW mode
            if(control_sample_gimbal_set_follow_mode(onboard) == true){
                /// control gimbal moving
                // control_sample_gimbal_set_move_angle(onboard, 40.f, 0.f, 170.f);
                control_sample_gimbal_set_move_angle(onboard, -40.f, 0.f, -65.f);
            }
        }
        break;

        case 7: // Set Gimbal to move rate in FOLLOW mode
        /* code */
        {
            /// set gimbal to FOLLOW mode
            if(control_sample_gimbal_set_follow_mode(onboard) == true){
                /// control gimbal moving
                control_sample_gimbal_set_move_rate(onboard, 10.f, 0, 10.f);
            }
        }
        break;

        case 8: // Set Gimbal to MAPPING mode
        /* code */
        {
            Gimbal_Interface::fw_version_t fw = onboard.get_gimbal_version();
            if((fw.x >= 7 && fw.y >= 8 && fw.z >= 1) || mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V1){
                control_sample_gimbal_set_mapping_mode(onboard);
            }
            else{
                GSDK_DebugWarning("This feature only support in case :\n\r"
                "- Use Mavlink gimbal protocol V1\n\r"
                "- Gimbal firmware version 7.8.1 and above\n\r")
            }
            
        }
        break;

        case 9: // Set Gimbal to Return Home
        /* code */
        {
            control_sample_gimbal_return_home(onboard);
        }
        break;
        
        case 10: // Set Gimbal Reboot
        /* code */
        {
            control_sample_gimbal_reboot(onboard);
        }
        break;
        default:
            break;
    }
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{
    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }
    }

    // end: for each input argument
    // Done!
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler(int sig)
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try {
        gimbal_interface_quit->handle_quit(sig);

    } catch (int error) {}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);

    } catch (int error) {}

    // end program here
    exit(0);
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    // This program uses throw, wrap one big try/catch here
    try {
        int result = gGimbal_sample(argc, argv);
        return result;

    } catch ( int error ) {
        fprintf(stderr, "mavlink_control threw exception %i \n", error);
        return error;
    }
}


/************************ (C) COPYRIGHT Gremsy *****END OF FILE****************/
