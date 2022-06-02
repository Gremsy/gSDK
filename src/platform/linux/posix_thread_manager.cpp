/*******************************************************************************
 * @file    posix_thread_manager.cpp
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

#include <unistd.h>
#include <string>
#include <sys/time.h>

#include "gimbal_interface.h"
#include "posix_thread_manager.h"

using namespace Linux;
using namespace GSDK::HAL;

namespace GSDK
{
    namespace HAL
    {
        uint64_t get_time_usec()
        {
            struct timeval _timestamp = { 0 };
            gettimeofday(&_timestamp, NULL);
            return _timestamp.tv_sec * 1000000 + _timestamp.tv_usec;
        }

        uint64_t get_time_msec()
        {
            struct timeval _timestamp = { 0 };
            gettimeofday(&_timestamp, NULL);
            return _timestamp.tv_sec * 1000 + _timestamp.tv_usec / 1000;
        }
    }
}

Posix_Thread::Posix_Thread(GSDK::Gimbal_Interface *gimbal, thread_type_t type)
{
    _gimbal = gimbal;
    _type   = type;
}

Posix_Thread::~Posix_Thread()
{
    stop_thread();
}

bool Posix_Thread::start_thread()
{
    if (!_is_stop) {
        fprintf(stderr, "Thread is already running!\n");
        return false;
    }

    int ret = -1;
    std::string info;

    switch (_type) {
        case SERIAL_READ_POLL:
            info = "READ";
            ret = pthread_create(&_tid, NULL, _read_poll, (void *)_gimbal);
            break;

        case SERIAL_WRITE_POLL:
            info = "WRITE";
            ret = pthread_create(&_tid, NULL, _write_poll, (void *)_gimbal);
            break;

        case PARAM_PROCESS:
            info = "PARAM PROCESS";
            ret = pthread_create(&_tid, NULL, _param_process, (void *)_gimbal);
            break;

        default:
            break;
    }

    if (ret != 0) {
        fprintf(stderr, "Failed to start %s thread! Resul %d\n", info.c_str(), ret);
        return false;
    }

    _is_stop = false;
    printf("Start %s thread!\n", info.c_str());
    return true;
}

bool Posix_Thread::stop_thread()
{
    if (_is_stop) {
        fprintf(stderr, "Thread is already stopped!\n");
        return false;
    }

    if (pthread_join(_tid, NULL)) {
        fprintf(stderr, "Failed to join thread\n");
        return false;
    }

    _is_stop = true;
    return true;
}

void Posix_Thread::delay_us(uint32_t time_us)
{
    usleep(time_us);
}

void Posix_Thread::delay_ms(uint32_t time_ms)
{
    usleep(time_ms * 1000);
}

void *Posix_Thread::_read_poll(void *arg)
{
    if (arg == nullptr) {
        fprintf(stderr, "Failed to run READ thread, NULL argument\n");
        return NULL;
    }

    GSDK::Gimbal_Interface *gimbal = (GSDK::Gimbal_Interface *)arg;
    gimbal->read_thread();
    return NULL;
}

void *Posix_Thread::_write_poll(void *arg)
{
    if (arg == nullptr) {
        fprintf(stderr, "Failed to run WRITE thread, NULL argument\n");
        return NULL;
    }

    GSDK::Gimbal_Interface *gimbal = (GSDK::Gimbal_Interface *)arg;
    gimbal->write_thread();
    return NULL;
}

void *Posix_Thread::_param_process(void *arg)
{
    if (arg == nullptr) {
        fprintf(stderr, "Failed to run PARAM PROCESS thread, NULL argument\n");
        return NULL;
    }

    GSDK::Gimbal_Interface *gimbal = (GSDK::Gimbal_Interface *)arg;
    gimbal->param_process();
    return NULL;
}

Posix_Mutex::Posix_Mutex()
{
    pthread_mutex_init(&_mutex, NULL);
}

Posix_Mutex::~Posix_Mutex()
{
    pthread_mutex_destroy(&_mutex);
}

void Posix_Mutex::lock()
{
    pthread_mutex_lock(&_mutex);
}

void Posix_Mutex::free()
{
    pthread_mutex_unlock(&_mutex);
}

Posix_Event::Posix_Event()
{
    pthread_mutex_init(&_mutex, NULL);
    pthread_cond_init(&_condition, NULL);
}

Posix_Event::~Posix_Event()
{
    pthread_mutex_destroy(&_mutex);
    pthread_cond_destroy(&_condition);
}

void Posix_Event::notify()
{
    // Lock mutex
    pthread_mutex_lock(&_mutex);
    pthread_cond_signal(&_condition);
    // Unlock mutex
    pthread_mutex_unlock(&_mutex);
}

bool Posix_Event::wait_ms(uint32_t time_ms)
{
    struct timespec time_wait = { 0 };
    // get current time
    clock_gettime(CLOCK_REALTIME, &time_wait);
    time_wait.tv_nsec += time_ms * 1000000;
    // Lock mutex
    pthread_mutex_lock(&_mutex);

    // Wait
    bool ret = !(pthread_cond_timedwait(&_condition, &_mutex, &time_wait) == ETIMEDOUT);

    // Unlock mutex
    pthread_mutex_unlock(&_mutex);

    return ret;
}

void Posix_Event::wait()
{
    // Lock mutex
    pthread_mutex_lock(&_mutex);
    pthread_cond_wait(&_condition, &_mutex);
    // Unlock mutex
    pthread_mutex_unlock(&_mutex);
}
