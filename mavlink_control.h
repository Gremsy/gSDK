/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    mavlink_control.h
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-021-2018
 * @brief   This file contains expand of gMavlink
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

using std::string;
using namespace std;

#include <ardupilotmega/mavlink.h>

#include "gimbal_interface.h"
#include "serial_port.h"


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// int main(int argc, char **argv);
int gGimbal_sample(int argc, char **argv);
void gGimbal_control_sample(Gimbal_Interface &gimbal_interface);
void gGimbal_displays(Gimbal_Interface &gimbal_interface);
void commands(Gimbal_Interface &gimbal_interface);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);

// quit handler
Gimbal_Interface *gimbal_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );

