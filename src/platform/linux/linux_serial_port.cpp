/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file linux_serial_port.cpp
 *
 * @brief Serial interface functions
 *
 * Functions for opening, closing, reading and writing via serial ports
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <sys/ioctl.h> // ioctl() call defenitions

#include "linux_serial_port.h"

using namespace Linux;

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Linux_Serial_Port::Linux_Serial_Port(const char *device, uint32_t baudrate) :
    _device(device)
{
    _baudrate = baudrate;
}

Linux_Serial_Port::~Linux_Serial_Port()
{
    _close_port();
}

size_t Linux_Serial_Port::serial_read(uint8_t *buf, size_t maxlen)
{
    return _read_port(buf, maxlen);
}

size_t Linux_Serial_Port::serial_write(const uint8_t *buf, size_t len)
{
    return _write_port(buf, len);
}

void Linux_Serial_Port::start()
{
    _start_port();
}

void Linux_Serial_Port::stop()
{
    _close_port();
}

void Linux_Serial_Port::_start_port()
{
    printf("OPEN PORT\n");
    _fd = _open_port(_device);

    // Check success
    if (_fd == -1) {
        fprintf(stderr, "failure, could not open port.\n");
        throw EXIT_FAILURE;
    }

    if (!_setup_port(_baudrate, 8, 1, false, false)) {
        fprintf(stderr, "failure, could not configure port.\n");
        _close_port();
        throw EXIT_FAILURE;
    }

    if (_fd <= 0) {
        fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", _device, _baudrate);
        _close_port();
        throw EXIT_FAILURE;
    }

    printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1).\n\n", _device, _baudrate);
    _device_status = true;
}

void Linux_Serial_Port::_close_port()
{
    printf("CLOSE PORT\n");

    if (close(_fd)) {
        fprintf(stderr, "WARNING: Error on port close.\n");
    }

    _device_status = false;
    _fd            = -1;
    printf("\n");
}

// ------------------------------------------------------------------------------
//   Helper Function - Open Serial Port File Descriptor
// ------------------------------------------------------------------------------
// Where the actual port opening happens, returns file descriptor 'fd'
int Linux_Serial_Port::_open_port(const char *port)
{
    // Open serial port
    // O_RDWR - Read and write
    // O_NOCTTY - Ignore special chars like CTRL-C
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    // Check for Errors
    if (fd == -1) {
        /* Could not open the port. */
        return (-1);
    }

    // Finalize
    else {
        fcntl(fd, F_SETFL, 0);
    }

    // Done!
    return fd;
}

// ------------------------------------------------------------------------------
//   Helper Function - Setup Serial Port
// ------------------------------------------------------------------------------
// Sets configuration, flags, and baud rate
bool Linux_Serial_Port::_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
    // Check file descriptor
    if (!isatty(_fd)) {
        fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", _fd);
        return false;
    }

    // Read file descritor configuration
    struct termios config;

    if (tcgetattr(_fd, &config) < 0) {
        fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", _fd);
        return false;
    }

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                        ONOCR | OFILL | OPOST);
    #ifdef OLCUC
    config.c_oflag &= ~OLCUC;
    #endif
    #ifdef ONOEOT
    config.c_oflag &= ~ONOEOT;
    #endif
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 10; // was 0

    // Get the current options for the port
    ////struct termios options;
    ////tcgetattr(fd, &options);

    // Apply baudrate
    switch (baud) {
        case 1200:
            if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }

            break;

        case 1800:
            cfsetispeed(&config, B1800);
            cfsetospeed(&config, B1800);
            break;

        case 9600:
            cfsetispeed(&config, B9600);
            cfsetospeed(&config, B9600);
            break;

        case 19200:
            cfsetispeed(&config, B19200);
            cfsetospeed(&config, B19200);
            break;

        case 38400:
            if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }

            break;

        case 57600:
            if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }

            break;

        case 115200:
            if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }

            break;

        // These two non-standard (by the 70'ties ) rates are fully supported on
        // current Debian and Mac OS versions (tested since 2010).
        case 460800:
            if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }

            break;

        case 921600:
            if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }

            break;

        default:
            fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
            return false;
            break;
    }

    // Finally, apply the configuration
    if (tcsetattr(_fd, TCSAFLUSH, &config) < 0) {
        fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", _fd);
        return false;
    }

    // Done!
    return true;
}

// ------------------------------------------------------------------------------
//   Read Port
// ------------------------------------------------------------------------------
int Linux_Serial_Port::_read_port(uint8_t *buf, size_t maxlen)
{
    return read(_fd, buf, maxlen);
}

// ------------------------------------------------------------------------------
//   Write Port
// ------------------------------------------------------------------------------
int Linux_Serial_Port::_write_port(const uint8_t *buf, size_t len)
{
    return write(_fd, buf, len);
}
