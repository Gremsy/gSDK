/*******************************************************************************
 * @file    posix_thread_manager.h
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

#ifndef POSIX_THREAD_MANAGER_H_
#define POSIX_THREAD_MANAGER_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <pthread.h>

#include "gsdk_thread_manager.h"

namespace Linux
{
    class Posix_Thread : public GSDK::HAL::gSDK_Thread
    {
    public:
        Posix_Thread(GSDK::Gimbal_Interface *gimbal, GSDK::HAL::thread_type_t type);
        ~Posix_Thread() override;

        bool start_thread() override;
        bool stop_thread() override;

        void delay_us(uint32_t time_us) override;
        void delay_ms(uint32_t time_ms) override;

    private:
        pthread_t _tid;

        static void *_read_poll(void *arg);
        static void *_write_poll(void *arg);
        static void *_param_process(void *arg);
    };

    class Posix_Mutex : public GSDK::HAL::gSDK_Mutex
    {
    public:
        Posix_Mutex();
        ~Posix_Mutex() override;

        void lock() override;
        void free() override;

    private:
        pthread_mutex_t _mutex;
    };

    class Posix_Event : public GSDK::HAL::gSDK_Event
    {
    public:
        Posix_Event();
        ~Posix_Event() override;

        void notify() override;
        bool wait_ms(uint32_t time_ms) override;
        void wait() override;

    private:
        pthread_mutex_t _mutex;
        pthread_cond_t _condition;
    };
    
} // namespace GSDK


#endif // POSIX_THREAD_MANAGER_H_
