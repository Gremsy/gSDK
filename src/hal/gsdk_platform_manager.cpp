/*******************************************************************************
 * @file    gsdk_platform_manager.cpp
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    May-25-2022
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

#include <cstdio>

#include "gsdk_platform_manager.h"

#ifdef __linux__
    #include "linux_serial_port.h"
    #include "posix_thread_manager.h"
#endif

using namespace GSDK::HAL;

gSDK_Thread *gSDK_Platform_Manager::add_thread(Gimbal_Interface *gimbal, thread_type_t type)
{
    gSDK_Thread *thread(nullptr);

    /* Add your platform implementation here */
    #ifdef __linux__
    thread = new Linux::Posix_Thread(gimbal, type);
    #endif

    if (thread == nullptr) {
        fprintf(stderr, "Failed to create new thread.\n");
    }

    return thread;
}

gSDK_Serial_Manager *gSDK_Platform_Manager::add_serial(const char *device, uint32_t baudrate)
{
    gSDK_Serial_Manager *serial(nullptr);

    /* Add your platform implementation here */
    #ifdef __linux__
    serial = new Linux::Linux_Serial_Port(device, baudrate);
    #endif

    if (serial == nullptr) {
        fprintf(stderr, "Failed to add serial device.\n");
    }

    return serial;    
}

gSDK_Mutex *gSDK_Platform_Manager::create_mutex()
{
    gSDK_Mutex *mutex(nullptr);

    /* Add your platform implementation here */
    #ifdef __linux__
    mutex = new Linux::Posix_Mutex();
    #endif

    if (mutex == nullptr) {
        fprintf(stderr, "Failed to create new mutex.\n");
    }

    return mutex;
}

gSDK_Event *gSDK_Platform_Manager::create_event()
{
    gSDK_Event *event(nullptr);

    /* Add your platform implementation here */
    #ifdef __linux__
    event = new Linux::Posix_Event();
    #endif

    if (event == nullptr) {
        fprintf(stderr, "Failed to create new event.\n");
    }

    return event;
}
