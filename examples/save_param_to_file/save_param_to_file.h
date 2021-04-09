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


// quit handler
Gimbal_Interface *gimbal_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );
