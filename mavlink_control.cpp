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

    STATE_CHECK_FIRMWARE_VERSION,
    STATE_SETTING_GIMBAL,
    STATE_SETTING_MAVLINK_MESSAGE,

    STATE_SET_GIMBAL_OFF,
    STATE_SET_GIMBAL_ON,

    STATE_SET_GIMBAL_FOLLOW_MODE,
    STATE_MOVE_GIMBAL_ANGLE_FOLLOW,
    STATE_MOVE_GIMBAL_RATE_FOLLOW,

    STATE_SET_GIMBAL_LOCK_MODE,
    STATE_MOVE_GIMBAL_ANGLE_LOCK,
    STATE_MOVE_GIMBAL_RATE_LOCK,

    STATE_SWITCH_TO_RC,
    STATE_CONTROL_WITH_RC,

    STATE_MOVE_TO_ZERO,

    STATE_SET_GIMBAL_REBOOT,

    STATE_DONE
};

struct sdk_process_t {
    sdk_process_state_t state = STATE_IDLE;
    uint64_t            timeout_ms;
};

/* Private variable ----------------------------------------------------------*/
static sdk_process_t sdk;
static Serial_Port *serial_port_quit;
static Gimbal_Interface *gimbal_interface_quit;

/* Private prototype ---------------------------------------------------------*/
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);
void quit_handler(int sig);
void gGimbal_control_sample(Gimbal_Interface &onboard);
void gGimbal_displays(Gimbal_Interface &api);

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
    #ifdef _USE_MAVLINK_GIMBAL_V1
    Gimbal_Interface gimbal_interface(&serial_port, 1, MAV_COMP_ID_ONBOARD_COMPUTER, Gimbal_Interface::MAVLINK_GIMBAL_V1);
    #else
    Gimbal_Interface gimbal_interface(&serial_port, 1, MAV_COMP_ID_ONBOARD_COMPUTER, Gimbal_Interface::MAVLINK_GIMBAL_V2);
    #endif
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
        if (gimbal_interface.present()) {
            // Reset time
            sdk.timeout_ms = get_time_msec();
            // Sample control
            gGimbal_control_sample(gimbal_interface);
            // Uncomment line below to dispay sample value
            // gGimbal_displays(gimbal_interface);

        } else if (get_time_msec() - sdk.timeout_ms > 2000) {
            /* Reset state */
            sdk.state = STATE_IDLE;
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

// --------------------------------------------------------------------------
//   Paser gimbal info
// --------------------------------------------------------------------------
void gGimbal_displays(Gimbal_Interface &api)
{
    // /*--------------------------------------------------------------------------
    //   GET A MESSAGE
    // --------------------------------------------------------------------------*/
    // printf("READ SOME MESSAGES \n");
    // printf("\n");
    // gimbal_status_t gimbal_status = api.get_gimbal_status();
    // printf("Got message gimbal status \n");
    // if(gimbal_status.state == GIMBAL_STATE_OFF)
    // {
    //     printf("Gimbal's status is OFF!\n");
    // }
    // else if(gimbal_status.state == GIMBAL_STATE_ON)
    // {
    //       printf("Gimbal is operating\n");
    // }
    // else if(gimbal_status.state == GIMBAL_STATE_INIT)
    // {
    //     printf("Gimbal is busy!\n");
    // }
    // else if(gimbal_status.state == GIMBAL_STATE_ERROR)
    // {
    //     printf("Gimbal's status is error!\n");
    // }
    // mavlink_raw_imu_t imu = api.get_gimbal_raw_imu();
    // imu.time_usec = api.get_gimbal_time_stamps().raw_imu;
    // printf("Got message RAW IMU.\n");
    // printf("\traw imu: time: %lu, xacc:%d, yacc:%d, zacc:%d, xgyro:%d, xgyro:%d, xgyro:%d(raw)\n",
    //                                                 (unsigned long)imu.time_usec,
    //                                                 imu.xacc,
    //                                                 imu.yacc,
    //                                                 imu.zacc,
    //                                                 imu.xgyro,
    //                                                 imu.ygyro,
    //                                                 imu.zgyro);
    // mavlink_mount_orientation_t mnt_orien = api.get_gimbal_attitude();
    // mnt_orien.time_boot_ms = api.get_gimbal_time_stamps().mount_orientation;
    // printf("Got message Mount orientation.\n");
    // printf("\torientation: time: %lu, p:%f, r:%f, y:%f (degree)\n",   (unsigned long)mnt_orien.time_boot_ms,
    //                                                                     mnt_orien.pitch,
    //                                                                     mnt_orien.roll,
    //                                                                     mnt_orien.yaw);
    // mavlink_mount_status_t mnt_status = api.get_gimbal_mount_status();
    // uint64_t mnt_status_time_stamp = api.get_gimbal_time_stamps().mount_status;
    // printf("Got message Mount status \n");
    // if(api.get_gimbal_config_mavlink_msg().enc_type_send)
    // {
    //     printf("\tEncoder Count: time: %lu, p:%d, r:%d, y:%d (Resolution 2^16)\n", (unsigned long)mnt_status_time_stamp,
    //                                                         mnt_status.pointing_a,
    //                                                         mnt_status.pointing_b,
    //                                                         mnt_status.pointing_c);
    // }
    // else
    // {
    //     printf("\tEncoder Angle: time: %lu, p:%d, r:%d, y:%d (Degree)\n", (unsigned long)mnt_status_time_stamp,
    //                                                         mnt_status.pointing_a,
    //                                                         mnt_status.pointing_b,
    //                                                         mnt_status.pointing_c);
    // }
    // gimbal_config_axis_t setting = api.get_gimbal_config_tilt_axis();
    // printf("\tSETTING TILT: dir %d, speed_follow: %d, speed_control: %d\n",
    //                                                         setting.dir,
    //                                                         setting.speed_follow,
    //                                                         setting.speed_control);
    // gimbal_motor_control_t tilt;
    // gimbal_motor_control_t roll;
    // gimbal_motor_control_t pan;
    // uint8_t output_filter, gyro_filter, gain;
    // api.get_gimbal_motor_control(tilt, roll, pan, gyro_filter, output_filter, gain);
    // printf("\tMOTOR_CONTROL: GYRO: %d, OUT %d, GAIN %d\n", gyro_filter, output_filter, gain);
    // printf("\tTILT  stiff %d, hold: %d\n" , tilt.stiffness, tilt.holdstrength);
    // printf("\tROLL  stiff %d, hold: %d\n" , roll.stiffness, roll.holdstrength);
    // printf("\tPAN   stiff %d, hold: %d\n" , pan.stiffness, pan.holdstrength);
    // printf("\n");
}

// ------------------------------------------------------------------------------
//   This example will demonstrate how to set gimbal mode
//      and control gimbal in angle and speed mode
// ------------------------------------------------------------------------------
void gGimbal_control_sample(Gimbal_Interface &onboard)
{
    switch (sdk.state) {
        case STATE_IDLE: {
                sdk.state = STATE_CHECK_FIRMWARE_VERSION;
            }
            break;

        case STATE_CHECK_FIRMWARE_VERSION: {
                Gimbal_Interface::fw_version_t fw = onboard.get_gimbal_version();
                printf("FW Version: %d.%d.%d.%s\n", fw.x, fw.y, fw.z, fw.type);

                // This firmware only apply for the firmware version from v7.x.x or above
                if (fw.x >= 7 && fw.y >= 5) {
                    sdk.state = STATE_SETTING_GIMBAL;

                } else {
                    printf("DO NOT SUPPORT FUNCTIONS. Please check the firmware version\n");
                    printf("1. MOTOR CONTROL\n");
                    printf("2. AXIS CONFIGURATION\n");
                    printf("3. MAVLINK MSG RATE CONFIGURATION\n");
                }
            }
            break;

        case STATE_SETTING_GIMBAL: {
                // Setting axis for control. see the struct gimbal_config_axis_t
                Gimbal_Interface::gimbal_config_axis_t config = { 0 };
                config = { Gimbal_Interface::DIR_CCW, 50, 20, 100, 20, 2 };   // Tilt
                onboard.set_gimbal_config_tilt_axis(config);
                config = { Gimbal_Interface::DIR_CCW, 50, 20, 0, 0, 2 };    // Roll
                onboard.set_gimbal_config_roll_axis(config);
                config = { Gimbal_Interface::DIR_CCW, 50, 20, 100, 20, 2 };  // Yaw
                onboard.set_gimbal_config_pan_axis(config);
                // Motor control likes: Stiffness, holdstrength, gyro filter, output filter and gain
                // Uncomment block below to configure gimbal motor
                /*                 Gimbal_Interface::gimbal_motor_control_t tilt = { 50, 30 };
                                Gimbal_Interface::gimbal_motor_control_t roll = { 60, 30 };
                                Gimbal_Interface::gimbal_motor_control_t pan = { 70, 30 };
                                onboard.set_gimbal_motor_control( tilt, roll, pan, 2, 3, 120 ); */
                sdk.state = STATE_SETTING_MAVLINK_MESSAGE;
            }
            break;

        case STATE_SETTING_MAVLINK_MESSAGE: {
                uint8_t enc_value_rate = 10;
                uint8_t orien_rate = 10;
                uint8_t imu_rate = 10;
                printf("Set encoder messages rate: %dHz\n", enc_value_rate);
                onboard.set_msg_encoder_rate(enc_value_rate);
                printf("Set mount orientation messages rate: %dHz\n", orien_rate);
                onboard.set_msg_mnt_orient_rate(orien_rate);
                printf("Set gimbal device attitude status messages rate: %dHz\n", orien_rate);
                onboard.set_msg_attitude_status_rate(orien_rate);
                printf("Set raw imu messgaes rate: %dHz\n", imu_rate);
                onboard.set_msg_raw_imu_rate(imu_rate);
                printf("Set gimbal send raw encoder value.\n");
                onboard.set_gimbal_encoder_type_send(true);

                if (onboard.request_gimbal_device_info() == Gimbal_Protocol::SUCCESS) {
                    printf("Request gimbal device information successfully!\n");
                    sdk.state = STATE_SET_GIMBAL_OFF;

                } else {
                    fprintf(stderr, "Could not request gimbal device information!\n");
                }
            }
            break;

        case STATE_SET_GIMBAL_OFF: {
                // Check gimbal is on
                if (onboard.get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_ON) {
                    // Turn off
                    if (onboard.set_gimbal_motor(Gimbal_Interface::TURN_OFF) == Gimbal_Protocol::SUCCESS) {
                        printf("TURN GIMBAL OFF Successfully!\n");
                        sdk.state = STATE_SET_GIMBAL_ON;

                    } else {
                        fprintf(stderr, "Could not TURN GIMBAL OFF!\n");
                    }

                } else if (onboard.get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_OFF) {
                    sdk.state = STATE_SET_GIMBAL_ON;
                }
            }
            break;

        case STATE_SET_GIMBAL_ON: {
                // Check gimbal is off
                if (onboard.get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_OFF) {
                    // Turn on gimbal
                    if (onboard.set_gimbal_motor(Gimbal_Interface::TURN_ON) == Gimbal_Protocol::SUCCESS) {
                        printf("TURN GIMBAL ON Sucessfully!\n");
                        sdk.state = STATE_SET_GIMBAL_FOLLOW_MODE;

                    } else {
                        fprintf(stderr, "Could not TURN GIMBAL ON!\n");
                    }

                } else if (onboard.get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_ON) {
                    sdk.state = STATE_SET_GIMBAL_FOLLOW_MODE;
                }
            }
            break;

        case STATE_SET_GIMBAL_FOLLOW_MODE: {
                Gimbal_Protocol::result_t res = onboard.set_gimbal_follow_mode_sync();

                if (res == Gimbal_Protocol::SUCCESS) {
                    printf("Set gimbal to FOLLOW MODE Successfully!\n");
                    sdk.state = STATE_MOVE_GIMBAL_ANGLE_FOLLOW;

                } else {
                    fprintf(stderr, "Could not set gimbal to FOLLOW MODE! Result code: %d\n", res);
                }
            }
            break;

        case STATE_MOVE_GIMBAL_ANGLE_FOLLOW: {
                // Target attitude
                float setpoint_pitch = 40.f;
                float setpoint_roll  = 0.f;
                float setpoint_yaw   = 170.f;
                printf("[FOLLOW] Move gimbal to Pitch - Roll - Yaw: %.2f - %.2f - %.2f\n", setpoint_pitch, setpoint_roll, setpoint_yaw);
                Gimbal_Protocol::result_t res = onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw);

                if (res == Gimbal_Protocol::SUCCESS) {
                    printf("\tSend command successfully!\n");
                    attitude<float> attitude;

                    do {
                        attitude = onboard.get_gimbal_attitude();
                        printf("\tGimbal attitude Pitch - Roll - Yaw: %.2f - %.2f - %.2f\n", attitude.pitch, attitude.roll, attitude.yaw);
                        usleep(500000);
                    } while (fabsf(attitude.pitch - setpoint_pitch) > 0.1f &&
                             fabsf(attitude.roll - setpoint_roll) > 0.1f &&
                             fabsf(attitude.yaw - setpoint_yaw) > 0.1f);

                    sdk.state = STATE_MOVE_GIMBAL_RATE_FOLLOW;

                } else {
                    fprintf(stderr, "\tCould not control gimbal! Result code: %d\n", res);
                }
            }
            break;

        case STATE_MOVE_GIMBAL_RATE_FOLLOW: {
                attitude<float> attitude;
                printf("\tReturn home\n");
                Gimbal_Protocol::result_t res = onboard.set_gimbal_reset_mode(Gimbal_Protocol::GIMBAL_RESET_MODE_PITCH_AND_YAW);

                if (res == Gimbal_Protocol::SUCCESS) {
                    /* Wait for returning home */
                    do {
                        usleep(500000);
                    } while (fabsf(attitude.pitch) > 0.1f &&
                             fabsf(attitude.roll) > 0.1f &&
                             fabsf(attitude.yaw) > 0.1f);

                } else {
                    fprintf(stderr, "\tCoudld not return home! Result code: %d\n", res);
                    break;
                }

                float pitch_rate = 1.f;
                printf("\tGimbal pitch up at rate %.2fdeg/s\n", pitch_rate);
                res = onboard.set_gimbal_rotation_rate_sync(pitch_rate, 0.f, 0.f);

                if (res == Gimbal_Protocol::SUCCESS) {
                    usleep(5000000);    // Move in 5s
                    onboard.set_gimbal_rotation_rate_sync(0.f, 0.f, 0.f);   // Stop

                } else {
                    fprintf(stderr, "\tCoudld not control pitch rate! Result code: %d\n", res);
                    break;
                }

                float yaw_rate = 1.f;
                printf("\tGimbal yaw right at rate %.2fdeg/s\n", yaw_rate);
                res = onboard.set_gimbal_rotation_rate_sync(0.f, 0.f, yaw_rate);

                if (res == Gimbal_Protocol::SUCCESS) {
                    usleep(5000000);    // Move in 5s
                    onboard.set_gimbal_rotation_rate_sync(0.f, 0.f, 0.f);   // Stop

                } else {
                    fprintf(stderr, "\tCoudld not control yaw rate! Result code: %d\n", res);
                    break;
                }

                sdk.state = STATE_SET_GIMBAL_LOCK_MODE;
            }
            break;

        case STATE_SET_GIMBAL_LOCK_MODE: {
                Gimbal_Protocol::result_t res = onboard.set_gimbal_lock_mode_sync();

                if (res == Gimbal_Protocol::SUCCESS) {
                    printf("Set gimbal to LOCK MODE Successfully!\n");
                    sdk.state = STATE_MOVE_GIMBAL_ANGLE_LOCK;

                } else {
                    fprintf(stderr, "Could not set gimbal to LOCK MODE! Result code: %d\n", res);
                }
            }
            break;

        case STATE_MOVE_GIMBAL_ANGLE_LOCK: {
                // Target attitude
                float setpoint_pitch = 40.f;
                float setpoint_roll  = 0.f;
                float setpoint_yaw   = 170.f;
                printf("[LOCK] Move gimbal to Pitch - Roll - Yaw: %.2f - %.2f - %.2f\n", setpoint_pitch, setpoint_roll, setpoint_yaw);
                Gimbal_Protocol::result_t res = onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw);

                if (res == Gimbal_Protocol::SUCCESS) {
                    printf("\tSend command successfully!\n");
                    attitude<float> attitude;

                    do {
                        attitude = onboard.get_gimbal_attitude();
                        printf("\tGimbal attitude Pitch - Roll - Yaw: %.2f - %.2f - %.2f\n", attitude.pitch, attitude.roll, attitude.yaw);
                        usleep(500000);
                    } while (fabsf(attitude.pitch - setpoint_pitch) > 0.1f &&
                             fabsf(attitude.roll - setpoint_roll) > 0.1f &&
                             fabsf(attitude.yaw - setpoint_yaw) > 0.1f);

                    sdk.state = STATE_MOVE_GIMBAL_RATE_LOCK;

                } else {
                    fprintf(stderr, "\tCould not control gimbal! Result code: %d\n", res);
                }
            }
            break;

        case STATE_MOVE_GIMBAL_RATE_LOCK: {
                attitude<float> attitude;
                printf("\tReturn home\n");
                Gimbal_Protocol::result_t res = onboard.set_gimbal_reset_mode(Gimbal_Protocol::GIMBAL_RESET_MODE_PITCH_AND_YAW);

                if (res == Gimbal_Protocol::SUCCESS) {
                    /* Wait for returning home */
                    do {
                        usleep(500000);
                    } while (fabsf(attitude.pitch) > 0.1f &&
                             fabsf(attitude.roll) > 0.1f &&
                             fabsf(attitude.yaw) > 0.1f);

                } else {
                    fprintf(stderr, "\tCoudld not return home! Result code: %d\n", res);
                    break;
                }

                float pitch_rate = 1.f;
                printf("\tGimbal pitch up at rate %.2fdeg/s\n", pitch_rate);
                res = onboard.set_gimbal_rotation_rate_sync(pitch_rate, 0.f, 0.f);

                if (res == Gimbal_Protocol::SUCCESS) {
                    usleep(5000000);    // Move in 5s
                    onboard.set_gimbal_rotation_rate_sync(0.f, 0.f, 0.f);   // Stop

                } else {
                    fprintf(stderr, "\tCoudld not control pitch rate! Result code: %d\n", res);
                    break;
                }

                float yaw_rate = 1.f;
                printf("\tGimbal yaw to East at rate %.2fdeg/s\n", yaw_rate);
                res = onboard.set_gimbal_rotation_rate_sync(0.f, 0.f, yaw_rate);

                if (res == Gimbal_Protocol::SUCCESS) {
                    usleep(5000000);    // Move in 5s
                    onboard.set_gimbal_rotation_rate_sync(0.f, 0.f, 0.f);   // Stop

                } else {
                    fprintf(stderr, "\tCoudld not control yaw rate! Result code: %d\n", res);
                    break;
                }

                sdk.state = STATE_MOVE_TO_ZERO;
            }
            break;

        case STATE_SWITCH_TO_RC: {
                Gimbal_Protocol::result_t res = onboard.set_gimbal_rc_input_sync();

                if (res == Gimbal_Protocol::SUCCESS) {
                    printf("Set Gimbal to RC Input mode Successfully!\n");
                    sdk.state = STATE_CONTROL_WITH_RC;

                } else {
                    fprintf(stderr, "\tCoudld not switch Gimbal to RC Input mode! Result code: %d\n", res);
                }
            }
            break;

        case STATE_CONTROL_WITH_RC: {
                attitude<float> attitude = onboard.get_gimbal_attitude();
                printf("\tGimbal attitude Pitch - Roll - Yaw: %.2f - %.2f - %.2f\n", attitude.pitch, attitude.roll, attitude.yaw);
                usleep(1000000);
            }
            break;

        case STATE_MOVE_TO_ZERO: {
                Gimbal_Protocol::result_t res = onboard.set_gimbal_follow_mode_sync();

                if (res == Gimbal_Protocol::SUCCESS) {
                    printf("Set gimbal to FOLLOW MODE Successfully!\n");

                } else {
                    fprintf(stderr, "Could not set gimbal to FOLLOW MODE! Result code: %d\n", res);
                }

                printf("\tReturn home\n");
                res = onboard.set_gimbal_reset_mode(Gimbal_Protocol::GIMBAL_RESET_MODE_PITCH_AND_YAW);

                if (res == Gimbal_Protocol::SUCCESS) {
                    attitude<float> attitude;

                    /* Wait for returning home */
                    do {
                        attitude = onboard.get_gimbal_attitude();
                        usleep(500000);
                    } while (fabsf(attitude.pitch) > 0.1f &&
                             fabsf(attitude.roll) > 0.1f &&
                             fabsf(attitude.yaw) > 0.1f);

                    sdk.state = STATE_SET_GIMBAL_REBOOT;

                } else {
                    fprintf(stderr, "\tCoudld not return home! Result code: %d\n", res);
                    break;
                }
            }
            break;

        case STATE_SET_GIMBAL_REBOOT: {
                printf("STATE_SET_GIMBAL_REBOOT!\n");
                if (onboard.set_gimbal_reboot() == Gimbal_Protocol::SUCCESS) {
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
