/*******************************************************************************
 * @file    gsdk_platform_manager.h
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    May-30-2022
 * @brief   This file contains API for gimbal interface
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

#ifndef GSDK_PLATFORM_MANAGER_H_
#define GSDK_PLATFORM_MANAGER_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "gsdk_types.h"
#include "gsdk_serial_manager.h"
#include "gsdk_thread_manager.h"

// ----------------------------------------------------------------------------------
//   Platform Manager Class
// ----------------------------------------------------------------------------------
namespace GSDK
{
    // Forward declaration
    class Gimbal_Interface;

    namespace HAL
    {
        class gSDK_Platform_Manager
        {
        public:
            gSDK_Platform_Manager(gSDK_Platform_Manager const&) = delete;
            void operator=(gSDK_Platform_Manager const&) = delete;

            static gSDK_Platform_Manager &get_platform()
            {
                static gSDK_Platform_Manager _instance;
                return _instance;
            }

            gSDK_Thread *add_thread(Gimbal_Interface *gimbal, thread_type_t type);
            gSDK_Serial_Manager *add_serial(const char *device, uint32_t baudrate);
            gSDK_Mutex *create_mutex();
            gSDK_Event *create_event();

        private:
            gSDK_Platform_Manager() {}

        };

        uint64_t get_time_usec();
        uint64_t get_time_msec();

    } // namespace HAL
        
} // namespace GSDK


#endif // GSDK_PLATFORM_MANAGER_H_
