/*******************************************************************************
 * @file    gsdk_thread_manager.h
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-21-2018
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

#ifndef GSDK_THREAD_MANAGER_H_
#define GSDK_THREAD_MANAGER_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <cstdlib>
#include <cstdint>

// ----------------------------------------------------------------------------------
//   THREAD Manager Class
// ----------------------------------------------------------------------------------

namespace GSDK
{
    namespace HAL
    {
        enum thread_type_t {
            SERIAL_READ_POLL,
            SERIAL_WRITE_POLL,
            PARAM_PROCESS,
        };

        class gSDK_Thread
        {
        public:
            gSDK_Thread();
            virtual ~gSDK_Thread();

            /*! @note How to use
            *  In order to provide platform crossable gSDK library,
            *  we abstract this class as a serial hardware level.
            *
            *  @note function descriptions:
            *
            *  bool start_thread();
            *  @brief After calling this function, gSDK_Thread should be able to
            *  start the thread.
            *
            *  bool stop_thread();
            *  @brief After calling this function, gSDK_Thread should stop the thread
            *
            *  void pause_us(uint32_t time_us);
            *  void pause_ms(uint32_t time_ms);
            *  @brief After calling this function, current thread should be paused.
            *
            *  @attention
            *  when writting and reading data, there might have multi-thread problems.
            *  Abstract class gsdk_serial_manager did not consider these issue.
            *  Please be careful when you are going to implement send and serial_read
            *  funtions.
            *
            *  @note
            *  we strongly suggest you to inherit this class in your own file, not just
            *  implement
            *  it in gsdk_serial_manager.cpp or inside this class
            *
            * */

            virtual bool start_thread() = 0;
            virtual bool stop_thread() = 0;

            virtual void pause_us(uint32_t time_us) = 0;
            virtual void pause_ms(uint32_t time_ms) = 0;

            bool is_thread_stop() const
            {
                return _is_stop;
            }

            bool get_type() const
            {
                return _type;
            }

        protected:
            bool _is_stop = true;
            thread_type_t _type;
        };

        // Mutex
        class gSDK_Mutex
        {
        public:
            gSDK_Mutex();
            virtual ~gSDK_Mutex();

            virtual void lock() = 0;
            virtual void free() = 0;
        };

        // Thread sync/comm
        class gSDK_Event
        {
        public:
            gSDK_Event();
            virtual ~gSDK_Event();

            virtual void notify() = 0;
            virtual bool wait_ms(uint32_t time_ms) = 0;
            virtual void wait() = 0;
        };

    } // HAL

} // GSDK

#endif // GSDK_THREAD_MANAGER_H_
