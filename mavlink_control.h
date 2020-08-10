/*******************************************************************************
 * @file    mavlink_control.h
 * @author  The GremsyCo
 * @version V2.3.0
 * @date    August-21-2018
 * @brief   This file contains variables for serial handle
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

/************************ (C) COPYRIGHT Gremsy *****END OF FILE****************/
