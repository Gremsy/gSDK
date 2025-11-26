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
 #define _USE_MAVLINK_GIMBAL_V1

#define _TIMEOUT                    30
#define _TIMEOUT_RETURN_HOME        3
/* Private Typedef------------------------------------------------------------*/

enum sdk_process_state_t {
    STATE_IDLE,
    STATE_CONNECTED,
    STATE_PROCESS,
    STATE_DONE
};

enum gimbal_axis{
    AXIS_PITCH = 0x01,
    AXIS_ROLL,
    AXIS_PAN,
};

enum type_of_gimbal
{
    TWO_AXIS = 0x01,
    THREE_AXIS,
};

enum mount_mode
{
    TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT = 0x01,
    TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT,
    THREE_AXIS_GIMBAL_MOUNT_NORMAL_MODE = 0x06,
    THREE_AXIS_GIMBAL_MOUNT_INVERTED_MODE,
    OFFSET = 5
};


struct sdk_process_t {
    sdk_process_state_t state = STATE_IDLE;
    uint64_t            timeout_ms;
};

struct gimbal_mode_t{
    type_of_gimbal gimbal_type;
    uint8_t mnt_mode;  /*Reference by 'mount_mode'*/
};

/* Private variable ----------------------------------------------------------*/


static sdk_process_t sdk;
static Generic_Port *port;
static Gimbal_Interface *gimbal_interface;

static gimbal_mode_t gimbal_mode;
static bool is_boot_mode = false;

#ifdef _USE_MAVLINK_GIMBAL_V1
static Gimbal_Interface::MAVLINK_PROTO    mav_gimbal_proto = Gimbal_Interface::MAVLINK_GIMBAL_V1;
#else
static Gimbal_Interface::MAVLINK_PROTO    mav_gimbal_proto = Gimbal_Interface::MAVLINK_GIMBAL_V2;
#endif

/* Private prototype ---------------------------------------------------------*/
int  gGimbal_sample(int argc, char **argv);
void gGimbal_displays(Gimbal_Interface *onboard);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);
void quit_handler(int sig);

//Function
static void setting_sample_gimbal_setup_param_startup(Gimbal_Interface *onboard);
static void setting_sample_gimbal_setup_message_rate(Gimbal_Interface *onboard);
static void setting_sample_gimbal_set_follow_param(Gimbal_Interface *onboard);
static void setting_sample_gimbal_set_stiffness_param(Gimbal_Interface *onboard);

static void control_sample_gimbal_process(Gimbal_Interface *onboard, Generic_Port *port);

static void control_sample_gimbal_off(Gimbal_Interface *onboard);
static void control_sample_gimbal_on(Gimbal_Interface *onboard);
static void control_sample_gimbal_change_mount_mode(Gimbal_Interface *onboard);
static void control_sample_gimbal_config_follow_para(Gimbal_Interface *onboard);
static void control_sample_gimbal_get_gimbal_information(Gimbal_Interface *onboard);

static bool control_sample_gimbal_set_lock_mode(Gimbal_Interface *onboard);
static bool control_sample_gimbal_set_follow_mode(Gimbal_Interface *onboard);
static void control_sample_gimbal_set_mapping_mode(Gimbal_Interface *onboard);
static void control_sample_gimbal_set_move_angle(Gimbal_Interface *onboard, float pitch_angle, float roll_angle, float yaw_angle);
static void control_sample_gimbal_set_move_rate(Gimbal_Interface *onboard, float pitch_rate, float roll_rate, float yaw_rate, uint8_t duration);

static void control_sample_gimbal_return_home(Gimbal_Interface *onboard);
static void control_sample_gimbal_reboot(Gimbal_Interface *onboard);

static bool upgrade_firmware(Gimbal_Interface *onboard, Generic_Port *port);

static void monitor_attitude_imu_encoder(Gimbal_Interface *onboard, uint8_t duration);

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
#if PORT_TYPE_serial
    port = new Serial_Port(uart_name, baudrate);
#elif PORT_TYPE_udp
    port = new UDP_Port("0.0.0.0", 14550);
#else
    #error "Unsupported PORT_TYPE"
#endif
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
    // Gimbal_Interface gimbal_interface(&serial_port, 1, MAV_COMP_ID_ONBOARD_COMPUTER, mav_gimbal_proto, MAVLINK_COMM_0);
    gimbal_interface = new Gimbal_Interface(port, 1, MAV_COMP_ID_ONBOARD_COMPUTER, mav_gimbal_proto, MAVLINK_COMM_0);   
    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    signal(SIGINT, quit_handler);
    /*
     * Start the port and Gimbal_interface
     * This is where the port is opened, and read and write threads are started.
     */
    port->start();
    gimbal_interface->start();

    /// Process data
    while (!gimbal_interface->get_flag_exit()) {
        if(sdk.state == STATE_IDLE){
            if (gimbal_interface->present()) {
                sdk.state = STATE_CONNECTED;
            }
        } else if(sdk.state == STATE_CONNECTED) {
            setting_sample_gimbal_setup_param_startup(gimbal_interface);

            sdk.state = STATE_PROCESS;
        } else {
            control_sample_gimbal_process(gimbal_interface, port);

        }

        usleep(1000);   // Run at 1kHz
    }

    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------
    /*
     * Now that we are done we can stop the threads and close the port
     */
    gimbal_interface->stop();
    port->stop();
    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------
    // woot!
    return 0;
}

void gGimbal_displays(Gimbal_Interface *onboard){

    Gimbal_Interface::fw_version_t fw = onboard->get_gimbal_version();
    GSDK_DebugInfo("Gimbal Firmware version is %d.%d.%d.%s\n", fw.x, fw.y, fw.z, fw.type);

    GSDK_DebugInfo("You selected gimbal mount mode ");
    std :: string mode ;
    switch (gimbal_mode.mnt_mode)
    {
    case TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT:
        GSDK_DebugInfo("Two axis Roll-Tilt mount\n");
        break;
    case TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT:
        GSDK_DebugInfo("Two axis Pan-Tilt Mount\n");
        break;
    case THREE_AXIS_GIMBAL_MOUNT_INVERTED_MODE:
        GSDK_DebugInfo("Three axis Inverted Mount\n");
        break;
    case THREE_AXIS_GIMBAL_MOUNT_NORMAL_MODE:
        GSDK_DebugInfo("Three axis Normal Mount\n");
        break;
    default:
        break;
    }


    uint8_t gimbal_mode = onboard->get_gimbal_mode();

    GSDK_DebugInfo("Gimbal mode: %d\n",gimbal_mode);

    uint8_t gimbal_state = onboard->get_gimbal_status().state;

    GSDK_DebugInfo("Gimbal state: %d\n",gimbal_state);

    Gimbal_Interface::imu_t my_imu =  onboard->get_gimbal_raw_imu();

    GSDK_DebugInfo("Raw imu:  xacc:%d, yacc:%d, zacc:%d, xgyro:%d, xgyro:%d, xgyro:%d(raw)\n",
                                                    my_imu.accel.x,
                                                    my_imu.accel.y,
                                                    my_imu.accel.z,
                                                    my_imu.gyro.x,
                                                    my_imu.gyro.y,
                                                    my_imu.gyro.z);
    Attitude_t<float> myattitude;
    myattitude= onboard->get_gimbal_attitude();

    GSDK_DebugInfo("Gimbal attitude Pitch - Roll - Yaw: (%.2f) - (%.2f) - (%.2f)\n" ,myattitude.pitch, myattitude.roll, myattitude.yaw);
    Attitude_t<int16_t> myencoder;
    myencoder = onboard->get_gimbal_encoder();

    GSDK_DebugInfo("Gimbal encoder Pitch - Roll - Yaw: (%d) - (%d) - (%d)\n" ,myencoder.pitch, myencoder.roll, myencoder.yaw);

    Gimbal_Interface::gimbal_config_axis_t new_config = onboard->get_gimbal_config_tilt_axis();

    GSDK_DebugInfo("Config follow TILT: Dir: %d -- Speed control: %d -- Smooth control: %d -- Smooth follow: %d -- Window follow: %d\n" 
        , new_config.dir , new_config.speed_control ,new_config.smooth_control, new_config.smooth_follow, new_config.window_follow);
    
    new_config = onboard->get_gimbal_config_roll_axis();

    GSDK_DebugInfo("Config follow ROLL: Dir: %d -- Speed control: %d -- Smooth control: %d -- Smooth follow: %d -- Window follow: %d\n" 
        , new_config.dir , new_config.speed_control ,new_config.smooth_control, new_config.smooth_follow, new_config.window_follow);

    new_config = onboard->get_gimbal_config_pan_axis();

    GSDK_DebugInfo("Config follow PAN: Dir: %d -- Speed control: %d -- Smooth control: %d -- Smooth follow: %d -- Window follow: %d\n" 
        , new_config.dir , new_config.speed_control ,new_config.smooth_control, new_config.smooth_follow, new_config.window_follow);

    Gimbal_Interface :: limit_angle_t limit =  onboard->get_limit_angle_pitch();

    GSDK_DebugInfo("Pitch max: %d -- Pitch min: %d\n" , limit.angle_max, limit.angle_min);

    limit =  onboard->get_limit_angle_roll();

    GSDK_DebugInfo("Roll max: %d -- Roll min: %d\n" , limit.angle_max, limit.angle_min);

    limit =  onboard->get_limit_angle_yaw();

    GSDK_DebugInfo("Yaw max: %d -- Yaw min: %d\n" , limit.angle_max, limit.angle_min);


    Gimbal_Interface:: gimbal_motor_control_t tilt;
    Gimbal_Interface:: gimbal_motor_control_t roll;
    Gimbal_Interface:: gimbal_motor_control_t pan;
    uint8_t gyro_filter ;
    uint8_t output_filter;
    onboard->get_gimbal_motor_control(tilt,roll,pan,gyro_filter,output_filter);

    GSDK_DebugInfo("Tilt hold strength: %d -- Tilt stiffness: %d\n" , tilt.holdstrength, tilt.stiffness);
    GSDK_DebugInfo("Roll hold strength: %d -- Roll stiffness: %d\n" , roll.holdstrength, roll.stiffness);
    GSDK_DebugInfo("Pan hold strength: %d -- Pan stiffness: %d\n" , pan.holdstrength, pan.stiffness);
    GSDK_DebugInfo("Gyro filter: %d\n",gyro_filter );
    GSDK_DebugInfo("Output filter: %d\n" , output_filter);

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
        gimbal_interface->handle_quit(sig);

    } catch (int error) {}

    // serial port
    try {
        port->stop();

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


// ------------------------------------------------------------------------------
//   Helper function
// ------------------------------------------------------------------------------
static void setting_sample_set_limit_angle(Gimbal_Interface *onboard){

        Gimbal_Interface :: limit_angle_t limit ;

        //setitng limit for pan axis
        limit.angle_max = 320;
        limit.angle_min = -320;
        while (onboard->set_limit_angle_yaw(limit) != Gimbal_Protocol::SUCCESS)
        {
            printf("Try to set yaw limit again!\r\n");
            usleep(1000);
        }
        printf("Set yaw max: %d -- Set yaw min: %d\n" , limit.angle_max, limit.angle_min);

        // setting limit for titl axis
        limit.angle_max = 45;
        limit.angle_min = -45;
        while (onboard->set_limit_angle_pitch(limit) != Gimbal_Protocol::SUCCESS)
        {
            printf("Try to set titl limit again!\r\n");
            usleep(1000);
        }
        printf("Set yaw max: %d -- Set yaw min: %d\n" , limit.angle_max, limit.angle_min);

        // setting limit for roll axis
        limit.angle_max = 40;
        limit.angle_min = -40;
        while (onboard->set_limit_angle_roll(limit) != Gimbal_Protocol::SUCCESS)
        {
            printf("Try to set roll limit again!\r\n");
            usleep(1000);
        }
        printf("Set roll max: %d -- Set roll min: %d\n" , limit.angle_max, limit.angle_min);
}

static void get_input_angle(float &pitch,float &roll, float &yaw);

static void get_input_rate(float &pitch,float &roll, float &yaw, uint8_t &duration);

static void get_input_duration(uint8_t &duration);

static void get_encoder_mode(uint8_t &mode);

static void control_sample_gimbal_process(Gimbal_Interface *onboard, Generic_Port *port){

    if (onboard->present() == false) {
        return;
    }

    int number = 0;

    printf("\33[39m\n\r Please Enter number [0-15] to select Gimbal control mode\n\r");
    printf("\t 0.   OFF Gimbal\n\r");
    printf("\t 1.   ON Gimbal\n\r");
    printf("\t 2.   Change mount mode\n\r");
    printf("\t 3.   Gimbal information\n\r");
    printf("\t 4.   Config gimbal follow parameter\n\r");
    printf("\t 5.   Set Gimbal to LOCK mode\n\r");
    printf("\t 6.   Set Gimbal to move angle in LOCK mode\n\r");
    printf("\t 7.   Set Gimbal to move rate in LOCK mode\n\r");
    printf("\t 8.   Set Gimbal to FOLLOW mode\n\r");
    printf("\t 9.   Set Gimbal to move angle in FOLLOW mode\n\r");
    printf("\t 10.  Set Gimbal to move rate in FOLLOW mode\n\r");
    printf("\t 11.  Set Gimbal to MAPPING mode\n\r");
    printf("\t 12.  Set Gimbal to Return Home\n\r");\
    printf("\t 13.  Set Gimbal Reboot\n\r");
    printf("\t 14.  Upgrade Firmware\n\r");
    printf("\t 15.  Monitoring Encoder - Attitude - IMU\n\r");
    printf("\33[32mSelect ->  ");

    scanf("%d", &number);

    switch (number)
    {
        case 0: // OFF Gimbal

        {
            control_sample_gimbal_off(onboard);
        }
            break;

        case 1: // ON Gimbal

        {
            control_sample_gimbal_on(onboard);
        }
            break;

        case 2: //Change mount mode
        {
            control_sample_gimbal_change_mount_mode(onboard);
        }
            break;
        case 3: // Get gimbal information
        {
            control_sample_gimbal_get_gimbal_information(onboard);
        }
            break;
        case 4: // Config gimbal follow parameter
        {
            do
            {
               char input = 0;
               control_sample_gimbal_config_follow_para(onboard);
               printf("\t\n\rContinue (Y/N)?\n\r");
               scanf(" %c", &input);
               if(input != 'Y' && input != 'y'){
                    break;
               }

            } while (true);
        }
            break;
        case 5: // Set Gimbal to LOCK mode
        {
            control_sample_gimbal_set_lock_mode(onboard);
        }
            break;

        case 6: // Set Gimbal to move angle in LOCK mode
        {
            /// set gimbal to LOCK mode
            if(control_sample_gimbal_set_lock_mode(onboard) == true){
                float yaw;
                float pitch;
                float roll;
                get_input_angle(pitch,roll,yaw);
                control_sample_gimbal_set_move_angle(onboard,pitch,roll,yaw);
            }
        }
            break;

        case 7: // Set Gimbal to move rate in LOCK mode
        {
            // set gimbal to LOCK mode
            if(control_sample_gimbal_set_lock_mode(onboard) == true){
                // control gimbal moving
                float yaw_rate;
                float pitch_rate;
                float roll_rate;
                uint8_t duration;
                get_input_rate(pitch_rate,roll_rate,yaw_rate,duration);
                control_sample_gimbal_set_move_rate(onboard,pitch_rate,roll_rate,yaw_rate,duration);
            }
        }
            break;

        case 8: // Set Gimbal to FOLLOW mode
        {
            control_sample_gimbal_set_follow_mode(onboard);
        }
            break;

        case 9: // Set Gimbal to move angle in FOLLOW mode
        {
            /// set gimbal to FOLLOW mode
            if(control_sample_gimbal_set_follow_mode(onboard) == true){
                float yaw;
                float pitch;
                float roll;
                get_input_angle(pitch,roll,yaw);
                control_sample_gimbal_set_move_angle(onboard,pitch,roll,yaw);
            }
        }
        break;

        case 10: // Set Gimbal to move rate in FOLLOW mode
        {
            /// set gimbal to FOLLOW mode
            if(control_sample_gimbal_set_follow_mode(onboard) == true){
                /// control gimbal moving
                float yaw_rate;
                float pitch_rate;
                float roll_rate;
                uint8_t duration;
                get_input_rate(pitch_rate,roll_rate,yaw_rate,duration);
                control_sample_gimbal_set_move_rate(onboard,pitch_rate,roll_rate,yaw_rate,duration);
            }
        }
        break;
        case 11: // Set Gimbal to MAPPING mode
        {
            control_sample_gimbal_set_mapping_mode(onboard);
        }
        break;

        case 12: // Set Gimbal to Return Home
        {
            control_sample_gimbal_return_home(onboard);
        }
            break;
        
        case 13: // Set Gimbal Reboot
        /* code */
        {
            control_sample_gimbal_reboot(onboard);
        }
            break;
        case 14: // upgrade firmware
        /* code */
        {
            upgrade_firmware(onboard, port);
        }
            break;
        case 15: // Monitoring
        {
            uint8_t duration = 0;
            get_input_duration(duration);
            monitor_attitude_imu_encoder(onboard,duration);
        }
            break;
        default:
        {
        }
        break;
    }
}

static void setting_sample_gimbal_setup_param_startup(Gimbal_Interface *onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;

    if(mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V1){
        GSDK_DebugInfo("Use mavlink gimbal V1");
    }else{
        GSDK_DebugInfo("Use mavlink gimbal V2");
    }

    Gimbal_Interface::fw_version_t fw = onboard->get_gimbal_version();
    GSDK_DebugInfo("Gimbal Firmware version is %d.%d.%d.%s", fw.x, fw.y, fw.z, fw.type);

    if(fw.x * 100 + fw.y * 10 + fw.z <= 773){
        while(1){
            GSDK_DebugInfo("\n\rCurrent gSDK not support for this gimbal firmware version !!!\n\r"
            "Please contact to GREMSY support team for get correct version !!!\n\r"
            "Thank you !!!\n\r");

            usleep(1000000);
        }
    }
    onboard->set_gimbal_encoder_type_send(true);
    GSDK_DebugSuccess("Set gimbal send raw encoder value.\n");
    onboard->request_gimbal_device_info();
    GSDK_DebugSuccess("Request gimbal device information.\n");

    usleep(300000);

    {
        char gimbal_name[32] {0};
        onboard->get_gimbal_name(gimbal_name);                
        std::string gimbal_str(gimbal_name);
        std::string search_str = "AEVO";
        if (gimbal_str.find(search_str) != std::string::npos) {
            GSDK_DebugWarning("The AEVO gimbal needs to have the message rate set to function properly!\n\r");
            setting_sample_gimbal_setup_message_rate(onboard);
        }
        else{
            char number = 0;
            GSDK_DebugMsg("Please Enter y/n(yes or no) to setting gimbal mavlink message rate\n\r");
            scanf("%c", &number);
            getchar();
            if(number == 'Y'|| number == 'y')
            {
                setting_sample_gimbal_setup_message_rate(onboard);
            }
        }


        
    }

    {
        char number = 0;
        GSDK_DebugMsg("Please Enter y/n(yes or no) to setting gimbal stiffness - follow param\n\r");
        scanf("%c", &number);
        getchar();
        if(number == 'Y'|| number == 'y')
        {
            GSDK_DebugInfo("Setting gimbal stiffness - follow parametter");

            //Uncomment this if you want to set the stiffness param and follow param. 

            // setting_sample_gimbal_set_stiffness_param(onboard);

            // setting_sample_gimbal_set_follow_param(onboard);

        }
    }

    /// Please Uncomment to enable return home when change gimbal mode
    // do{
    //     /* code */
    //     res = onboard->set_gimbal_return_home_when_change_mode(true);
    // } while (res != Gimbal_Protocol::SUCCESS);
    int temp_gimbal_type; 
    do
    {
        GSDK_DebugMsg("\t\n\r Please Enter number to select type of gimbal\n\r");
        GSDK_DebugMsg("\t 1. Two axis\n\r");
        GSDK_DebugMsg("\t 2. Three axis\n\r");
        scanf("%d", &temp_gimbal_type);
        getchar();
    } while (!(temp_gimbal_type == (int)TWO_AXIS || temp_gimbal_type == (int)THREE_AXIS));
    gimbal_mode.gimbal_type = (enum type_of_gimbal) temp_gimbal_type;   
    
    if(gimbal_mode.gimbal_type == TWO_AXIS){

        do
        {
            GSDK_DebugMsg("Please Enter number to select two axis gimbal mount mode \n\r");
            GSDK_DebugMsg("\t 1. Roll-Tilt Mount\n\r");
            GSDK_DebugMsg("\t 2. Pan-Tilt Mount\n\r");
            scanf("%hhu", &gimbal_mode.mnt_mode);
            getchar();
        } while (!(gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT || gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT));
        
    }else if (gimbal_mode.gimbal_type == THREE_AXIS)
    {
        do
        {
            GSDK_DebugMsg("Please Enter number to select three axis gimbal mode \n\r");
            GSDK_DebugMsg("\t 1. Normal mode\n\r");
            GSDK_DebugMsg("\t 2. Inverted mode\n\r");
            scanf("%hhu", &gimbal_mode.mnt_mode);
            gimbal_mode.mnt_mode = gimbal_mode.mnt_mode + OFFSET;
        } while (!(gimbal_mode.mnt_mode == THREE_AXIS_GIMBAL_MOUNT_NORMAL_MODE || gimbal_mode.mnt_mode == THREE_AXIS_GIMBAL_MOUNT_INVERTED_MODE));
    }
    

    GSDK_DebugInfo("You selected gimbal mount mode ");
    std :: string mode ;
    switch (gimbal_mode.mnt_mode)
    {
    case TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT:
        GSDK_DebugInfo("Two axis Roll-Tilt mount\n");
        break;
    case TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT:
        GSDK_DebugInfo("Two axis Pan-Tilt Mount\n");
        break;
    case THREE_AXIS_GIMBAL_MOUNT_INVERTED_MODE:
        GSDK_DebugInfo("Three axis Inverted Mount\n");
        break;
    case THREE_AXIS_GIMBAL_MOUNT_NORMAL_MODE:
        GSDK_DebugInfo("Three axis Normal Mount\n");
        break;
    default:
        break;
    }
}

static void setting_sample_gimbal_setup_message_rate(Gimbal_Interface *onboard){
    char gimbal_name[32] {0};
    onboard->get_gimbal_name(gimbal_name);                
    std::string gimbal_str(gimbal_name);
    std::string search_str = "AEVO";
    GSDK_DebugInfo("Setting gimbal mavlink message\n");
    int enc_value_rate;
    int orien_rate;
    int imu_rate;
    set_encoder:
    GSDK_DebugMsg("Please Enter encoder messages rate (-1 for skip)\n");
    scanf("%d", &enc_value_rate);
    getchar();
    uint8_t timeout = 0;
    if (enc_value_rate > MAX_ENCODER_RATE)
    {
        GSDK_DebugWarning("The encoder messages rate exceeds the limit!");
    }
    else if(enc_value_rate >= 0)
    {
        int result = -1;
        do
        {
            usleep(10);
            result = onboard->set_msg_encoder_rate((Gimbal_Interface::rate_action_t)enc_value_rate) ;
            GSDK_DebugInfo("Try to set encoder messages rate: %dHz\r\n",enc_value_rate);

            if(timeout++ > _TIMEOUT){
                GSDK_DebugWarning("Cannot set up the encoder messages rate due to timeout!");
                break;
            }
        } while (result != 0);

        GSDK_DebugSuccess("Encoder message rate set successfully.\n");
    }else if(gimbal_str.find(search_str) != std::string::npos && enc_value_rate <= 0) {
        GSDK_DebugWarning("Please set message rate!");
        goto set_encoder;
    }
    set_orient:
    GSDK_DebugMsg("Please Enter mount orientation messages and attitude status messages rate (-1 for skip).\n\r");
    scanf("%d", &orien_rate);
    getchar();
    timeout = 0;
    if(orien_rate > MAX_ORIENTATION_RATE){
        GSDK_DebugWarning("The mount orientation messages rate exceeds the limit!");
    }
    else if(orien_rate >= 0) 
    {
        int result = -1;
        do
        {
            usleep(10);
            result = onboard->set_msg_mnt_orient_rate((Gimbal_Interface::rate_action_t)orien_rate);
            GSDK_DebugInfo("Try to set mount orientation messages rate: %dHz\r\n",orien_rate);
            
            if(timeout++ > _TIMEOUT){
                GSDK_DebugWarning("Cannot set up the mount orientation messages rate due to timeout!");
                break;
            }
        } while (result != 0);
        GSDK_DebugSuccess("Mount orientation messages rate set successfully.\n");

        result = -1;
        timeout = 0;
        do
        {
            usleep(10);
            result = onboard->set_msg_attitude_status_rate((Gimbal_Interface::rate_action_t)orien_rate);
            GSDK_DebugInfo("Try to set gimbal device attitude status messages rate: %dHz\r\n",orien_rate);

            if(timeout++ > _TIMEOUT){
                GSDK_DebugWarning("Cannot set up the attitude status messages rate due to timeout!");
                break;
            }
        } while (result != 0);
        GSDK_DebugSuccess("Gimbal device attitude status messages rate set successfully.\n");
    }else if(gimbal_str.find(search_str) != std::string::npos && orien_rate <= 0) {
        GSDK_DebugWarning("Please set message rate!");
        goto set_orient;
    }
    set_imu:
    GSDK_DebugMsg("Please Enter raw imu messages rate (-1 for skip)\n\r");
    scanf("%d", &imu_rate);
    getchar();
    timeout = 0;
    if (imu_rate > MAX_IMU_RAW_RATE)
    {
        GSDK_DebugWarning("The raw IMU rate exceeds the limit!");
    }
    else if(imu_rate >= 0)
    {
        int result = -1;
        do
        {
            usleep(10);
            result = onboard->set_msg_raw_imu_rate((Gimbal_Interface::rate_action_t)imu_rate);
            GSDK_DebugInfo("Try to set raw imu messages rate: %d\r\n",imu_rate);

            if(timeout++ > _TIMEOUT){
                GSDK_DebugWarning("Cannot set up the raw imu messages rate due to timeout!");
                break;
            }
        }  while (result != 0);
        GSDK_DebugSuccess("Raw imu messages rate set successfully.");
    }else if(gimbal_str.find(search_str) != std::string::npos && imu_rate <= 0) {
        GSDK_DebugWarning("Please set message rate!");
        goto set_imu;
    }
}

static void setting_sample_gimbal_set_follow_param(Gimbal_Interface *onboard){
    const int8_t tilt_dir               = Gimbal_Interface::DIR_CW;
    const uint8_t tilt_speed_control    = 30;
    const uint8_t tilt_smooth_control   = 80;
    const uint8_t tilt_smooth_follow    = 20;
    const uint8_t tilt_window_follow    = 5;

    const int8_t roll_dir               = Gimbal_Interface::DIR_CW;
    const uint8_t roll_speed_control    = 10;
    const uint8_t roll_smooth_control   = 80;
    const uint8_t roll_smooth_follow    = 0;
    const uint8_t roll_window_follow    = 0;

    const int8_t pan_dir                = Gimbal_Interface::DIR_CW;
    const uint8_t pan_speed_control     = 30;
    const uint8_t pan_smooth_control    = 80;
    const uint8_t pan_smooth_follow     = 20;
    const uint8_t pan_window_follow     = 10;

    Gimbal_Interface::gimbal_config_axis_t config = { 0 };
    config = { tilt_dir, tilt_speed_control, tilt_smooth_control, tilt_smooth_follow, tilt_window_follow };   // Tilt
    uint8_t timeout = 0;
    onboard->set_gimbal_config_tilt_axis(config);
    do
    {
        usleep(500000);
        Gimbal_Interface::gimbal_config_axis_t new_config = onboard->get_gimbal_config_tilt_axis();
        GSDK_DebugInfo("Config follow TILT: Dir: %u -- Speed control: %u -- Smooth control: %u -- Smooth follow: %u -- Window follow: %u\n" 
        , new_config.dir , new_config.speed_control ,new_config.smooth_control, new_config.smooth_follow, new_config.window_follow);
        
        if( new_config.dir            == config.dir             &&
            new_config.smooth_control == config.smooth_control  &&
            new_config.smooth_follow  == config.smooth_follow   &&
            new_config.speed_control  == config.speed_control   &&
            new_config.window_follow  == config.window_follow  )
        {
                GSDK_DebugSuccess("Set config for tilt successfully!");
                break;
        }

        if(timeout++ > _TIMEOUT){
            GSDK_DebugError("Cannot set up the tilt axis due to timeout!");
            break;
        }

        onboard->set_gimbal_config_tilt_axis(config);
        GSDK_DebugWarning("Try setting it again!!!");
    } while (1);
    
    config = { roll_dir, roll_speed_control, roll_smooth_control, roll_smooth_follow, roll_window_follow };    // Roll
    timeout = 0;
    onboard->set_gimbal_config_roll_axis(config);
    do
    {
        usleep(500000);
        Gimbal_Interface::gimbal_config_axis_t new_config = onboard->get_gimbal_config_roll_axis();
        GSDK_DebugInfo("Config follow ROLL: Dir: %u -- Speed control: %u -- Smooth control: %u -- Smooth follow: %u -- Window follow: %u\n" 
        , new_config.dir , new_config.speed_control ,new_config.smooth_control, new_config.smooth_follow, new_config.window_follow);
        
        if( new_config.dir            == config.dir             &&
            new_config.smooth_control == config.smooth_control  &&
            new_config.speed_control  == config.speed_control    )
        {
                GSDK_DebugSuccess("Set config for roll successfully!");
                break;
        }

        if(timeout++ > _TIMEOUT){
            GSDK_DebugError("Cannot set up the roll axis due to timeout!");
            break;
        }
        onboard->set_gimbal_config_roll_axis(config);
        GSDK_DebugWarning("Try setting it again!!!");
    } while (1);
    
    config = { pan_dir, pan_speed_control, pan_smooth_control, pan_smooth_follow, pan_window_follow };  // Yaw
    timeout = 0;
    onboard->set_gimbal_config_pan_axis(config);
    do
    {
        onboard->set_gimbal_config_pan_axis(config);
        usleep(500000);
        Gimbal_Interface::gimbal_config_axis_t new_config = onboard->get_gimbal_config_pan_axis();
        GSDK_DebugInfo("Config follow PAN: Dir: %u -- Speed control: %u -- Smooth control: %u -- Smooth follow: %u -- Window follow: %u\n" 
        , new_config.dir , new_config.speed_control ,new_config.smooth_control, new_config.smooth_follow, new_config.window_follow);
        
        if( new_config.dir            == config.dir             &&
            new_config.smooth_control == config.smooth_control  &&
            new_config.smooth_follow  == config.smooth_follow   &&
            new_config.speed_control  == config.speed_control   &&
            new_config.window_follow  == config.window_follow  )
        {
                GSDK_DebugSuccess("Set config for pan successfully!");
                break;
        }
        if(timeout++ > _TIMEOUT){
            GSDK_DebugError("Cannot set up the pan axis due to timeout!");
            break;
        }
        onboard->set_gimbal_config_pan_axis(config);
        GSDK_DebugWarning("Try setting it again!!!");
    } while (1);
    
}

static void setting_sample_gimbal_set_stiffness_param(Gimbal_Interface *onboard){
    const uint8_t tilt_stiffness        = 20;
    const uint8_t tilt_holdstrength     = 20;

    const uint8_t roll_stiffness        = 70;
    const uint8_t roll_holdstrength     = 70;

    const uint8_t pan_stiffness         = 60;
    const uint8_t pan_holdstrength      = 60;

    const uint8_t gyro_filter           = 4;
    const uint8_t output_filter         = 1;

    // Motor control likes: Stiffness, holdstrength, gyro filter, output filter and gain
    // Uncomment block below to configure gimbal motor
    Gimbal_Interface::gimbal_motor_control_t tilt = { tilt_stiffness, tilt_holdstrength };
    Gimbal_Interface::gimbal_motor_control_t roll = { roll_stiffness, roll_holdstrength };
    Gimbal_Interface::gimbal_motor_control_t pan = { pan_stiffness, pan_holdstrength };
    onboard->set_gimbal_motor_control( tilt, roll, pan, gyro_filter, output_filter ); 
    uint8_t timeout = 0;
    do
    {
        usleep(500000);
        Gimbal_Interface:: gimbal_motor_control_t n_tilt;
        Gimbal_Interface:: gimbal_motor_control_t n_roll;
        Gimbal_Interface:: gimbal_motor_control_t n_pan;
        uint8_t n_gyro_filter ;
        uint8_t n_output_filter;
        (void)onboard->get_gimbal_motor_control(n_tilt,n_roll,n_pan,n_gyro_filter,n_output_filter);
        if (n_tilt == tilt &&
            n_roll == roll &&
            n_pan  == pan  &&
            n_gyro_filter == gyro_filter &&
            n_output_filter == output_filter)
        {
            GSDK_DebugSuccess("Set config for motor successfully!");
            break;
        }
        
        if(timeout++ > _TIMEOUT){
            GSDK_DebugError("Cannot set up the motor due to timeout!");
            break;
        }

        GSDK_DebugWarning("Try setting it again!!!")
        onboard->set_gimbal_motor_control( tilt, roll, pan, gyro_filter, output_filter ); 
    } while (1);
    
}

static void control_sample_gimbal_off(Gimbal_Interface *onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    static uint8_t timeout = 0;

    // Check gimbal is on
    if (onboard->get_gimbal_status().state != Gimbal_Interface::GIMBAL_STATE_OFF) {
        do{
            res = onboard->set_gimbal_motor(Gimbal_Interface::TURN_OFF);
            usleep(500000);

            if(timeout++ > _TIMEOUT){
                GSDK_DebugError("Could not TURN GIMBAL OFF!\n");
                return;
            }

        }while (res != Gimbal_Protocol::SUCCESS);

        GSDK_DebugSuccess("TURN GIMBAL OFF Successfully!\n");

    } else if (onboard->get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_OFF) {
        GSDK_DebugWarning("Gimbal is OFF");
    }
}

static void control_sample_gimbal_on(Gimbal_Interface *onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    static uint8_t timeout = 0;

    // Check gimbal is off
    if (onboard->get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_OFF) {
        do{
            res = onboard->set_gimbal_motor(Gimbal_Interface::TURN_ON);
            usleep(500000);

            if(timeout++ > _TIMEOUT){
                GSDK_DebugError("Could not TURN GIMBAL ON!\n");
                return;
            }

        }while (res != Gimbal_Protocol::SUCCESS);

        GSDK_DebugSuccess("TURN GIMBAL ON Successfully!\n");

    } else if (onboard->get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_FOLLOW_MODE) {
        GSDK_DebugWarning("Gimbal is ON");
    }
}

static void control_sample_gimbal_change_mount_mode(Gimbal_Interface *onboard)
{
    int temp_gimbal_type; 
    do
    {
        GSDK_DebugMsg("\t\n\r Please Enter number to select type of gimbal\n\r");
        GSDK_DebugMsg("\t 1. Two axis\n\r");
        GSDK_DebugMsg("\t 2. Three axis\n\r");
        scanf("%d", &temp_gimbal_type);

    } while (!(temp_gimbal_type == (int)TWO_AXIS || temp_gimbal_type == (int)THREE_AXIS));
    gimbal_mode.gimbal_type = (enum type_of_gimbal) temp_gimbal_type;   
    if(gimbal_mode.gimbal_type == TWO_AXIS)
    {

        do
        {
            GSDK_DebugMsg("\t\n\r Please Enter number to select two axis gimbal mount mode \n\r");
            GSDK_DebugMsg("\t 1. Roll-Tilt Mount\n\r");
            GSDK_DebugMsg("\t 2. Pan-Tilt Mount\n\r");
            scanf("%hhu", &gimbal_mode.mnt_mode);
        } while (!(gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT || gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT));
        
    }
    
    else if (gimbal_mode.gimbal_type == THREE_AXIS)

    {
        do
        {
            GSDK_DebugMsg("\t\n\r Please Enter number to select three axis gimbal mode \n\r");
            GSDK_DebugMsg("\t 1. Normal mode\n\r");
            GSDK_DebugMsg("\t 2. Inverted mode\n\r");
            scanf("%hhu", &gimbal_mode.mnt_mode);
            gimbal_mode.mnt_mode = gimbal_mode.mnt_mode + OFFSET;
        } while (!(gimbal_mode.mnt_mode == THREE_AXIS_GIMBAL_MOUNT_NORMAL_MODE || gimbal_mode.mnt_mode == THREE_AXIS_GIMBAL_MOUNT_INVERTED_MODE));
    }
    

    GSDK_DebugInfo("You selected gimbal mount mode ");
    std :: string mode ;
    switch (gimbal_mode.mnt_mode)
    {
    case TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT:
        GSDK_DebugInfo("Two axis Roll-Tilt mount\n");
        break;
    case TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT:
        GSDK_DebugInfo("Two axis Pan-Tilt Mount\n");
        break;
    case THREE_AXIS_GIMBAL_MOUNT_INVERTED_MODE:
        GSDK_DebugInfo("Three axis Inverted Mount\n");
        break;
    case THREE_AXIS_GIMBAL_MOUNT_NORMAL_MODE:
        GSDK_DebugInfo("Three axis Normal Mount\n");
        break;
    default:
        break;
    }
    control_sample_gimbal_return_home(onboard);
}

static void control_sample_gimbal_config_follow_para(Gimbal_Interface *onboard){
    int axis;
    int dir;
    int speed_control  ;
    int smooth_control ;
    int smooth_follow  ;
    int window_follow  ;
    uint8_t max_axis, min_axis;
    if (gimbal_mode.mnt_mode ==  TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT)
    {

        GSDK_DebugMsg("Chose axis: \n");
        GSDK_DebugMsg("\t 1: Tilt\n");
        GSDK_DebugMsg("\t 2: Roll\n");
        GSDK_DebugMsg("\t 3: Reset parameter\n");
        GSDK_DebugMsg("\t Other: Exit\n");

        scanf("%d",&axis);

        if(!(axis == 1 || axis == 2 || axis == 3)){
            return;
        }

        if (axis == 3) axis = 4;

        
    }
    else if (gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT)
    {
        GSDK_DebugMsg("Chose axis: \n");
        GSDK_DebugMsg("\t 1: Tilt\n");
        GSDK_DebugMsg("\t 2: Pan\n");
        GSDK_DebugMsg("\t 3: Reset parameter\n");
        GSDK_DebugMsg("\t Other: Exit\n");

        scanf("%d",&axis);

        if(!(axis == 1 || axis == 2 || axis == 3)){
            return;
        }

        if (axis == 3) axis = 4;
        if (axis == 2) axis = 3;
    }
    else
    {
        GSDK_DebugMsg("Chose axis: \n");
        GSDK_DebugMsg("\t 1: Tilt\n");
        GSDK_DebugMsg("\t 2: Roll\n");
        GSDK_DebugMsg("\t 3: Pan\n");
        GSDK_DebugMsg("\t 4: Reset parameter\n");
        GSDK_DebugMsg("\t Other: Exit\n");

        scanf("%d",&axis);
        
        if(!(axis == 1 || axis == 2 || axis == 3 || axis == 4)){
            return;
        }
    }
    
    

    if(axis == 4){
        setting_sample_gimbal_set_follow_param(onboard);
        return;
    }

    GSDK_DebugInfo("You have chosen %d\n" , axis);

    do
    {
        GSDK_DebugMsg("\tDirection (0: Clockwise, 1: Counter clockwise): ");

        scanf("%d",&dir);

    } while (!(dir == 0 || dir == 1));
    
    

    GSDK_DebugMsg("\tSpeed control: ");

    scanf("%d",&speed_control);

    GSDK_DebugMsg("\tSmooth control: ");

    scanf("%d",&smooth_control);

    GSDK_DebugMsg("\tSmooth follow: ");

    scanf("%d",&smooth_follow);

    GSDK_DebugMsg("\tWindow follow: ");

    scanf("%d",&window_follow);

    GSDK_DebugInfo("You have chosen %d\n" , axis);
    
    Gimbal_Interface::gimbal_config_axis_t config = { 0 };
    config = { (int8_t)dir, (uint8_t)speed_control, (uint8_t)smooth_control, (uint8_t)smooth_follow, (uint8_t)window_follow};   
    int result = -1;
    uint8_t timeout = 0;
    if(axis == 1){
        // Tilt
        uint8_t timeout = 0;
        do
        {
            onboard->set_gimbal_config_tilt_axis(config);
            usleep(100000);
            Gimbal_Interface::gimbal_config_axis_t new_config = onboard->get_gimbal_config_tilt_axis();
            GSDK_DebugInfo("Config follow TILT: Dir: %u -- Speed control: %u -- Smooth control: %u -- Smooth follow: %u -- Window follow: %u\n" 
            , new_config.dir , new_config.speed_control ,new_config.smooth_control, new_config.smooth_follow, new_config.window_follow);
            
            if( new_config.dir            == config.dir             &&
                new_config.smooth_control == config.smooth_control  &&
                new_config.smooth_follow  == config.smooth_follow   &&
                new_config.speed_control  == config.speed_control   &&
                new_config.window_follow  == config.window_follow  )
            {
                    GSDK_DebugSuccess("Set config for tilt successfully!");
                    break;
            }

            if(timeout++ > _TIMEOUT){
                GSDK_DebugError("Cannot set up the tilt axis due to timeout!");
                break;
            }

            GSDK_DebugWarning("Try setting it again!!!");
        } while (1);

    }else if (axis == 2 )
    {
        // Roll
        do
        {
            onboard->set_gimbal_config_roll_axis(config);
            usleep(100000);
            Gimbal_Interface::gimbal_config_axis_t new_config = onboard->get_gimbal_config_roll_axis();
            GSDK_DebugInfo("Config follow ROLL: Dir: %u -- Speed control: %u -- Smooth control: %u -- Smooth follow: %u -- Window follow: %u\n" 
            , new_config.dir , new_config.speed_control ,new_config.smooth_control, new_config.smooth_follow, new_config.window_follow);
            
            if( new_config.dir            == config.dir             &&
                new_config.smooth_control == config.smooth_control  &&
                new_config.speed_control  == config.speed_control    )
            {
                    GSDK_DebugSuccess("Set config for roll successfully!");
                    break;
            }

            if(timeout++ > _TIMEOUT){
                GSDK_DebugError("Cannot set up the roll axis due to timeout!");
                break;
            }

            GSDK_DebugWarning("Try setting it again!!!");
        } while (1);
    
    }else if (axis == 3)
    {
        // Yaw
        do
        {
            onboard->set_gimbal_config_pan_axis(config);
            usleep(100000);
            Gimbal_Interface::gimbal_config_axis_t new_config = onboard->get_gimbal_config_pan_axis();
            GSDK_DebugInfo("Config follow PAN: Dir: %u -- Speed control: %u -- Smooth control: %u -- Smooth follow: %u -- Window follow: %u\n" 
            , new_config.dir , new_config.speed_control ,new_config.smooth_control, new_config.smooth_follow, new_config.window_follow);
            
            if( new_config.dir            == config.dir             &&
                new_config.smooth_control == config.smooth_control  &&
                new_config.smooth_follow  == config.smooth_follow   &&
                new_config.speed_control  == config.speed_control   &&
                new_config.window_follow  == config.window_follow  )
            {
                    GSDK_DebugSuccess("Set config for pan successfully!");
                    break;
            }
            if(timeout++ > _TIMEOUT){
                GSDK_DebugError("Cannot set up the pan axis due to timeout!");
                break;
            }

            GSDK_DebugWarning("Try setting it again!!!");
        } while (1);
    }else{
        GSDK_DebugError("SOME THING WRONG!!!");
    }
    

}

static void control_sample_gimbal_get_gimbal_information(Gimbal_Interface *onboard){
    gGimbal_displays(onboard);
}

static bool control_sample_gimbal_set_lock_mode(Gimbal_Interface *onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    uint8_t timeout = 0;

    do{
        res = onboard->set_gimbal_lock_mode_sync();
        
        if(timeout++ > _TIMEOUT){
            GSDK_DebugError("Could not set gimbal to LOCK MODE! Result code: %d\n", res);
            return false;
        }

    }while(res != Gimbal_Protocol::SUCCESS);

    GSDK_DebugSuccess("Set gimbal to LOCK MODE Successfully!\n");

    return true;
}

static bool control_sample_gimbal_set_follow_mode(Gimbal_Interface *onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    static uint8_t timeout = 0;

    do{
        res = onboard->set_gimbal_follow_mode_sync();

        if(timeout++ > _TIMEOUT){
            GSDK_DebugError("Could not set gimbal to FOLLOW MODE! Result code: %d\n", res);
            return false;
        }

    }while(res != Gimbal_Protocol::SUCCESS);

    GSDK_DebugSuccess("Set gimbal to FOLLOW MODE Successfully!\n");

    return true;
}

static void control_sample_gimbal_set_move_angle(Gimbal_Interface *onboard, float pitch_angle, float roll_angle, float yaw_angle){
    // Target attitude
    float setpoint_pitch = pitch_angle;
    float setpoint_roll  = roll_angle;
    float setpoint_yaw   = yaw_angle;
    GSDK_DebugInfo("Move gimbal to Pitch - Roll - Yaw: (%.2f) - (%.2f) - (%.2f)\n", setpoint_pitch, setpoint_roll, setpoint_yaw);
    Gimbal_Protocol::result_t res = onboard->set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw);
    if (res == Gimbal_Protocol::SUCCESS) {
        GSDK_DebugSuccess("\tSend command successfully!\n");
        Attitude_t<float> attitude;
        uint32_t gimbal_attitude_flag = onboard->get_gimbal_attitude_flag();
        uint8_t timeout = 0;
        do {

            usleep(500000);
            attitude = onboard->get_gimbal_attitude();
            
            res = onboard->set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw);

            if(gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT){
                if(mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V1) {
                    GSDK_DebugInfo("\tGimbal attitude Pitch - Roll: (%.2f) - (%.2f)\n", attitude.pitch, attitude.roll)
                    if((fabsf(attitude.pitch - setpoint_pitch)   < 0.5f)    &&
                        (fabsf(attitude.roll - setpoint_roll)    < 0.5f))
                    {
                        break;
                    }   
                } else if(mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V2){
                    GSDK_DebugInfo("\tGimbal attitude Pitch - Roll: (%.2f) - (%.2f)\n", attitude.eu_angle_forward.pitch, attitude.eu_angle_forward.roll);
                    if((fabsf(attitude.eu_angle_forward.pitch - setpoint_pitch)   < 0.5f)    &&
                       (fabsf(attitude.eu_angle_forward.roll - setpoint_roll)     < 0.5f))
                    {
                        break;
                    }   
                    
                }

            }else if (gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT)
            {
                /* Check for earth frame */
                if(gimbal_attitude_flag & GIMBAL_DEVICE_FLAGS_YAW_LOCK)
                {
                    if(mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V1){
                        GSDK_DebugInfo("\tGimbal attitude Pitch - Yaw: (%.2f) - (%.2f)\n", attitude.pitch, attitude.yaw);
                        if ((fabsf(attitude.pitch - setpoint_pitch)  < 0.5f)     &&
                            ((fabsf(attitude.yaw - setpoint_yaw)     < 0.5f)     || ((fabsf(attitude.yaw- setpoint_yaw) < 360.5f) && (fabsf(attitude.yaw - setpoint_yaw) > 359.5f))))
                        {
                            break;
                        }                        
                    }else{
                        GSDK_DebugInfo("\tGimbal attitude earth Pitch - Yaw: (%.2f) - (%.2f)\n", attitude.eu_angle_north.pitch, attitude.eu_angle_north.yaw);
                        GSDK_DebugInfo("\tGimbal attitude vehicle Pitch - Yaw: (%.2f) - (%.2f)\n", attitude.eu_angle_forward.pitch, attitude.eu_angle_forward.yaw);                    
                        if ((fabsf(attitude.eu_angle_north.pitch - setpoint_pitch)   < 0.5f)     &&
                            ((fabsf(attitude.eu_angle_north.yaw  - setpoint_yaw)     < 0.5f)     || 
                            ((fabsf(attitude.eu_angle_north.yaw - setpoint_yaw) < 360.5f) && (fabsf(attitude.eu_angle_north.yaw - setpoint_yaw) > 359.5f))))
                        {
                            break;
                        }  
                    }
                }
                /* Check for vehicle frame */
                else if(gimbal_attitude_flag & ~GIMBAL_DEVICE_FLAGS_YAW_LOCK)
                {
                    if(mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V1){
                        GSDK_DebugInfo("\tGimbal attitude Pitch - Yaw: (%.2f) - (%.2f)\n", attitude.pitch, attitude.yaw);
                        if ((fabsf(attitude.pitch - setpoint_pitch)  < 0.5f)     &&
                            ((fabsf(attitude.yaw - setpoint_yaw)     < 0.5f)     || 
                            ((fabsf(attitude.yaw- setpoint_yaw) < 360.5f) && (fabsf(attitude.yaw - setpoint_yaw) > 359.5f))))
                        {
                            break;
                        }                        
                    }else{                    
                        GSDK_DebugInfo("\tGimbal attitude earth Pitch - Yaw: (%.2f) - (%.2f)\n", attitude.eu_angle_north.pitch, attitude.eu_angle_north.yaw);
                        GSDK_DebugInfo("\tGimbal attitude vehicle Pitch - Yaw: (%.2f) - (%.2f)\n", attitude.eu_angle_forward.pitch, attitude.eu_angle_forward.yaw);
                        if ((fabsf(attitude.eu_angle_forward.pitch - setpoint_pitch)   < 0.5f)     &&
                            ((fabsf(attitude.eu_angle_forward.yaw  - setpoint_yaw)     < 0.5f)     || 
                            ((fabsf(attitude.eu_angle_forward.yaw - setpoint_yaw) < 360.5f) && (fabsf(attitude.eu_angle_forward.yaw - setpoint_yaw) > 359.5f))))
                            {
                                break;
                            }                      
                        }
                    }
            }else
            {
                /* Check for earth frame */
                if(gimbal_attitude_flag & GIMBAL_DEVICE_FLAGS_YAW_LOCK)
                {
                    if(mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V1){
                        GSDK_DebugInfo("\tGimbal attitude Pitch - Roll - Yaw: (%.2f) - (%.2f) - (%.2f)\n", attitude.pitch, attitude.roll, attitude.yaw);
                        if ((fabsf(attitude.roll - setpoint_roll)    < 0.5f)     &&
                            (fabsf(attitude.pitch - setpoint_pitch)  < 0.5f)     &&
                            ((fabsf(attitude.yaw - setpoint_yaw)     < 0.5f)     || ((fabsf(attitude.yaw- setpoint_yaw) < 360.5f) && (fabsf(attitude.yaw - setpoint_yaw) > 359.5f))))
                        {
                            break;
                        }                        
                    }else{
                        GSDK_DebugInfo("\tGimbal attitude earth Pitch - Roll - Yaw: (%.2f) - (%.2f) - (%.2f)\n", attitude.eu_angle_north.pitch, attitude.eu_angle_north.roll, attitude.eu_angle_north.yaw);
                        GSDK_DebugInfo("\tGimbal attitude vehicle Pitch - Roll - Yaw: (%.2f) - (%.2f) - (%.2f)\n", attitude.eu_angle_forward.pitch, attitude.eu_angle_forward.roll, attitude.eu_angle_forward.yaw);                    
                        if ((fabsf(attitude.eu_angle_north.roll  - setpoint_roll)    < 0.5f)     &&
                            (fabsf(attitude.eu_angle_north.pitch - setpoint_pitch)   < 0.5f)     &&
                            ((fabsf(attitude.eu_angle_north.yaw  - setpoint_yaw)     < 0.5f)     || ((fabsf(attitude.eu_angle_north.yaw - setpoint_yaw) < 360.5f) && (fabsf(attitude.eu_angle_north.yaw - setpoint_yaw) > 359.5f))))
                        {
                            break;
                        }  
                    }
                }
                /* Check for vehicle frame */
                else if(gimbal_attitude_flag & ~GIMBAL_DEVICE_FLAGS_YAW_LOCK)
                {
                    if(mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V1){
                        GSDK_DebugInfo("\tGimbal attitude Pitch - Roll - Yaw: (%.2f) - (%.2f) - (%.2f)\n", attitude.pitch, attitude.roll, attitude.yaw);
                        if ((fabsf(attitude.roll - setpoint_roll)    < 0.5f)     &&
                            (fabsf(attitude.pitch - setpoint_pitch)  < 0.5f)     &&
                            ((fabsf(attitude.yaw - setpoint_yaw)     < 0.5f)     || ((fabsf(attitude.yaw- setpoint_yaw) < 360.5f) && (fabsf(attitude.yaw - setpoint_yaw) > 359.5f))))
                        {
                            break;
                        }                        
                    }else{                    
                        GSDK_DebugInfo("\tGimbal attitude earth Pitch - Roll - Yaw: (%.2f) - (%.2f) - (%.2f)\n", attitude.eu_angle_north.pitch, attitude.eu_angle_north.roll, attitude.eu_angle_north.yaw);
                        GSDK_DebugInfo("\tGimbal attitude vehicle Pitch - Roll - Yaw: (%.2f) - (%.2f) - (%.2f)\n", attitude.eu_angle_forward.pitch, attitude.eu_angle_forward.roll, attitude.eu_angle_forward.yaw);
                        if ((fabsf(attitude.eu_angle_forward.roll  - setpoint_roll)    < 0.5f)     &&
                            (fabsf(attitude.eu_angle_forward.pitch - setpoint_pitch)   < 0.5f)     &&
                            ((fabsf(attitude.eu_angle_forward.yaw  - setpoint_yaw)     < 0.5f)     || ((fabsf(attitude.eu_angle_forward.yaw - setpoint_yaw) < 360.5f) && (fabsf(attitude.eu_angle_forward.yaw - setpoint_yaw) > 359.5f))))
                            {
                                break;
                            }                      
                        }
                    }
            }

            if(timeout++ > _TIMEOUT){
                GSDK_DebugWarning("\tCould not move the gimbal correctly!");
                break;
            }
        } while (res == Gimbal_Protocol::SUCCESS);


    } else {
        GSDK_DebugError("\tCould not control gimbal! Result code: %d\n", res);
    }
}

static void control_sample_gimbal_set_move_rate(Gimbal_Interface *onboard, float pitch_rate, float roll_rate, float yaw_rate ,uint8_t duration){
    Attitude_t<float> attitude;
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    auto start = std::chrono::steady_clock::now();
    GSDK_DebugInfo("Gimbal move pitch - roll - yaw at rate: %.2fdeg/s - %.2fdeg/s - %.2fdeg/s\n", pitch_rate, roll_rate, yaw_rate);


    do {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start);
        if (elapsed.count() >= duration) {
            break;
        }
        attitude = onboard->get_gimbal_attitude();
        if(mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V1){
            if(gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT) {
                GSDK_DebugInfo("\tGimbal attitude Pitch - Roll: (%.2f) - (%.2f)\r\n",attitude.pitch,attitude.roll);
            }
            else if(gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT) {
                GSDK_DebugInfo("\tGimbal attitude Pitch - Yaw: (%.2f) - (%.2f)\r\n",attitude.pitch,attitude.yaw);
            }
        }else{
            if(gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT) {
                GSDK_DebugInfo("\tGimbal attitude Pitch - Roll in vehicle frame: (%.2f) - (%.2f)\n\n",attitude.eu_angle_forward.pitch,attitude.eu_angle_forward.roll);
            }
            else if(gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT) {
                GSDK_DebugInfo("\tGimbal attitude Pitch - Yaw in earth frame: (%.2f) - (%.2f)\n",attitude.eu_angle_north.pitch,attitude.eu_angle_north.yaw);
                GSDK_DebugInfo("\tGimbal attitude Pitch - Yaw in vehicle frame: (%.2f) - (%.2f)\n\n",attitude.eu_angle_forward.pitch,attitude.eu_angle_forward.yaw);
            }
            else {
                GSDK_DebugInfo("\tGimbal attitude Pitch - Roll - Yaw in earth frame: (%.2f) - (%.2f) - (%.2f)\n",attitude.eu_angle_north.pitch,attitude.eu_angle_north.roll,attitude.eu_angle_north.yaw);
                GSDK_DebugInfo("\tGimbal attitude Pitch - Roll - Yaw in vehicle frame: (%.2f) - (%.2f) - (%.2f)\n\n",attitude.eu_angle_forward.pitch,attitude.eu_angle_forward.roll,attitude.eu_angle_forward.yaw);                
            }
        }

        usleep(10000);
        res = onboard->set_gimbal_rotation_rate_sync(pitch_rate, roll_rate, yaw_rate);
    } while (res == Gimbal_Protocol::SUCCESS);

    /* Stop moving */
    do {
        usleep(500000);
        res = onboard->set_gimbal_rotation_rate_sync(0.f, 0.f, 0.f);
    } while (res != Gimbal_Protocol::SUCCESS);
    
}

static void control_sample_gimbal_return_home(Gimbal_Interface *onboard){
    Attitude_t<float> attitude;
    uint8_t timeout = 0;
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    
    /* Wait for returning home */
    do {
        res = onboard->set_gimbal_return_home_sync();
        usleep(500000);

        if(++timeout >= _TIMEOUT_RETURN_HOME){
            GSDK_DebugError("Could not set gimbal to RETURN HOME! Result code: %d\n", res);
            return;
        }

        attitude = onboard->get_gimbal_attitude();
    } while (   res != Gimbal_Protocol::SUCCESS &&
                (fabsf(attitude.pitch) > 0.5f ||
                fabsf(attitude.roll) > 0.5f ||
                fabsf(attitude.yaw) > 0.5f));
    GSDK_DebugSuccess("Gimbal RETURN HOME Successfully!\n");            
}

static void control_sample_gimbal_reboot(Gimbal_Interface *onboard) {

    Gimbal_Interface::fw_version_t fw = onboard->get_gimbal_version();

    uint32_t version = fw.x * 100 + fw.y * 10 + fw.z;

    /* Check firmware version */
    if(version > 0 && version < 787) {
        if (onboard->set_gimbal_reboot() == Gimbal_Protocol::SUCCESS) {
            while (onboard->get_gimbal_status().state != Gimbal_Interface::GIMBAL_STATE_ON)
                usleep(2000000);

        usleep(15000000); // wait to gimbal reboot  
        GSDK_DebugSuccess("Gimbal reboot is Successfully!\n");
        return;
        }   
    }
    else{
        if (onboard->set_gimbal_reboot(Gimbal_Interface::REBOOT_ACTION_REBOOT) == Gimbal_Protocol::SUCCESS) {
            bool reboot_started = false;
            bool reboot_done = false;
            int retries = 200; // 200 * 100ms = 20s timeout

            while (retries-- > 0) {
                auto status = onboard->get_gimbal_status();

                if (!reboot_started && status.state != Gimbal_Interface::GIMBAL_STATE_ON) {
                    reboot_started = true;
                    GSDK_DebugMsg("Gimbal reboot started (lost state ON)\n");
                }

                if (reboot_started && status.state == Gimbal_Interface::GIMBAL_STATE_ON) {
                    reboot_done = true;
                    break;
                }

                usleep(100000); 
            }

            if (reboot_done) {
                GSDK_DebugSuccess("Gimbal reboot success!\n");
            } else {
                GSDK_DebugError("Gimbal reboot timeout!\n");
            }
            return;
        }
    }
    GSDK_DebugError("Gimbal cannot reboot (command failed)!\n");
}

static void control_sample_gimbal_set_mapping_mode(Gimbal_Interface *onboard){
    Gimbal_Protocol::result_t res = Gimbal_Protocol::UNKNOWN;
    static uint8_t timeout = 0;

    do{
        res = onboard->set_gimbal_mapping_sync();
        
        if(++timeout >= _TIMEOUT){
            GSDK_DebugError("Could not set gimbal to MAPPING MODE! Result code: %d\n", res);
            return;
        }

    }while(res != Gimbal_Protocol::SUCCESS);

    GSDK_DebugSuccess("Set gimbal to MAPPING MODE Successfully!\n");
}
static bool upgrade_firmware(Gimbal_Interface *onboard, Generic_Port *port)
{
    std::string path = "";
    std::cout << "Enter a path: ";
    std::cin >> path;
    bool result = false;
    if (path == "")
    {
        GSDK_DebugError("The path is empty!");
        return false;
    }else
    {
        GSDK_DebugInfo("The path: %s",path.c_str());
    }
    
    onboard->stop();
    usleep(500000);
    port->stop();
    // const char * name = port->uart_name;
    Serial_Port * serial_port = dynamic_cast<Serial_Port *>(port);
    usleep(500000);
    {
        is_boot_mode = true;
        // Serial_Port _serial_port(uart_name,115200);
        Boot_loader boot_loader(serial_port, path);
        usleep(5000000);
        boot_loader.init();
        uint8_t timeout = 0;
        if(boot_loader.run())
        {
            result = true;
        }
        is_boot_mode = false;
    }
    port->start();
    onboard->start();
    return result;

}

static void monitor_attitude_imu_encoder(Gimbal_Interface *onboard,  uint8_t duration)
{
    Gimbal_Interface::imu_t my_imu;
    Attitude_t<float> myattitude;
    Attitude_t<int16_t> myencoder;

    auto start = std::chrono::steady_clock::now();
    uint8_t mode ;
    get_encoder_mode(mode);
    if (mode == 0 || mode == 1)
    {
        onboard->set_gimbal_encoder_type_send((mode == 0 ? false : true));
        usleep(500000);
    }
    
    do {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start);
        if (elapsed.count() >= duration) {
            break;
        }
        my_imu =  onboard->get_gimbal_raw_imu();
        GSDK_DebugInfo("\n\nRaw imu:  xacc:%d, yacc:%d, zacc:%d, xgyro:%d, xgyro:%d, xgyro:%d(raw)\n",
                                                        my_imu.accel.x,
                                                        my_imu.accel.y,
                                                        my_imu.accel.z,
                                                        my_imu.gyro.x,
                                                        my_imu.gyro.y,
                                                        my_imu.gyro.z);
        myattitude= onboard->get_gimbal_attitude();
        if(mav_gimbal_proto == Gimbal_Interface::MAVLINK_GIMBAL_V1){
        GSDK_DebugInfo("Gimbal attitude Pitch - Roll -Yaw: (%.2f) - (%.2f) - (%.2f)\n",myattitude.pitch,myattitude.roll,myattitude.yaw);       
        }
        else{
        GSDK_DebugInfo("Gimbal attitude Pitch - Roll - Yaw in earth frame: (%.2f) - (%.2f) - (%.2f)\n" , myattitude.eu_angle_north.pitch, myattitude.eu_angle_north.roll, myattitude.eu_angle_north.yaw);
        GSDK_DebugInfo("Gimbal attitude Pitch - Roll - Yaw in vehicle frame: (%.2f) - (%.2f) - (%.2f)\n" , myattitude.eu_angle_forward.pitch, myattitude.eu_angle_forward.roll, myattitude.eu_angle_forward.yaw);
        }
        myencoder = onboard->get_gimbal_encoder();

        GSDK_DebugInfo("Gimbal encoder Pitch - Roll - Yaw: (%d) - (%d) - (%d)\n" ,myencoder.pitch, myencoder.roll, myencoder.yaw);
        usleep(500000);
    }while (1);

    

}

static void get_input_angle(float &pitch,float &roll, float &yaw){
    if (gimbal_mode.gimbal_type == THREE_AXIS)
    {

        GSDK_DebugMsg("PITCH angle: ");
        scanf("%f" , &pitch);
        printf("\n");
        GSDK_DebugMsg("ROLL angle: ");
        scanf("%f" , &roll);
        printf("\n");
        GSDK_DebugMsg("YAW angle: ");
        scanf("%f" , &yaw);
    }
    else if (gimbal_mode.gimbal_type == TWO_AXIS && gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT)
    {
        GSDK_DebugMsg("PITCH angle: ");
        scanf("%f" , &pitch);
        printf("\n");
        GSDK_DebugMsg("ROLL angle: ");
        scanf("%f" , &roll);
        printf("\n");
        yaw = 0;
    }
    else if (gimbal_mode.gimbal_type == TWO_AXIS && gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT)
    {
        roll = 0;
        GSDK_DebugMsg("PITCH angle: ");
        scanf("%f" , &pitch);
        printf("\n");
        GSDK_DebugMsg("YAW angle: ");
        scanf("%f" , &yaw);
    }
    
}

static void get_input_rate(float &pitch_rate,float &roll_rate, float &yaw_rate, uint8_t &duration){
    if (gimbal_mode.gimbal_type == THREE_AXIS)
    {
        GSDK_DebugMsg("ROLL rate: ");
        scanf("%f" , &roll_rate);
        printf("\n");
        GSDK_DebugMsg("PITCH rate: ");
        scanf("%f" , &pitch_rate);
        printf("\n");
        GSDK_DebugMsg("YAW rate: ");
        scanf("%f" , &yaw_rate);
    }
    else if (gimbal_mode.gimbal_type == TWO_AXIS && gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_ROLL_TILT)
    {
        GSDK_DebugMsg("ROLL rate: ");
        scanf("%f" , &roll_rate);
        printf("\n");
        GSDK_DebugMsg("PITCH rate: ");
        scanf("%f" , &pitch_rate);
        printf("\n");
        yaw_rate = 0;
    }
    else if (gimbal_mode.gimbal_type == TWO_AXIS && gimbal_mode.mnt_mode == TWO_AXIS_GIMBAL_MOUNT_MODE_PAN_TILT)
    {
        roll_rate = 0;
        GSDK_DebugMsg("PITCH rate: ");
        scanf("%f" , &pitch_rate);
        printf("\n");
        GSDK_DebugMsg("YAW rate: ");
        scanf("%f" , &yaw_rate);
    }

    printf("\n");
    GSDK_DebugMsg("Duration (in seconds): ");
    scanf("%hhu" , &duration);
}

static void get_encoder_mode(uint8_t &mode)
{
    printf("\n");
    GSDK_DebugMsg("Enter encoder mode: 0 - Angle, 1 - Count , Else - skip");
    scanf("%hhu" , &mode);
}

static void get_input_duration(uint8_t &duration)
{
    printf("\n");
    GSDK_DebugMsg("Duration (in seconds): ");
    scanf("%hhu" , &duration);
}

static uint64_t get_time_usec(void) 
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);  // get directly from monotonic kernel (linux - WSL2)           
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000ULL);
}

static uint64_t get_time_msec()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static uint64_t get_time_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec;
}

static uint64_t get_time_min(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec / 60ULL;
}

static inline uint64_t time_since_us(uint64_t start_time)
{
    return get_time_usec() - start_time;
}

static inline uint64_t time_since_ms(uint64_t start_time)
{
    return get_time_msec() - start_time;
}

static inline uint64_t time_since_sec(uint64_t start_sec)
{
    return get_time_sec() - start_sec;
}

static inline uint64_t time_since_min(uint64_t start_min)
{
    return get_time_min() - start_min;
}


/************************ (C) COPYRIGHT Gremsy *****END OF FILE****************/
