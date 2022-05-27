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

#include "gimbal_interface.h"
#include "gsdk_thread_manager.h"

namespace GSDK
{
    namespace HAL
    {
        class Posix_Thread : public gSDK_Thread
        {
        public:
            Posix_Thread(const Gimbal_Interface *gimbal, thread_type_t type);
            ~Posix_Thread() override;

            bool start_thread() override;
            bool stop_thread() override;

            void pause_us(uint32_t time_us) override;
            void pause_ms(uint32_t time_ms) override;

        private:
            pthread_t _tid;
            const Gimbal_Interface *_gimbal;

            static void *_read_poll(void *arg);
            static void *_write_poll(void *arg);
            static void *_param_process(void *arg);
        };       
        
    } // namespace HAL
    
} // namespace GSDK


#endif // POSIX_THREAD_MANAGER_H_
