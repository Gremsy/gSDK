/*******************************************************************************
 * @file    gsdk_serial_manager.h
 * @author  The GremsyCo
 * @version V1.1.0
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

#ifndef GSDK_SERIAL_MANAGER_H_
#define GSDK_SERIAL_MANAGER_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <cstdlib>
#include <cstdint>

// ----------------------------------------------------------------------------------
//   Serial Manager Class
// ----------------------------------------------------------------------------------

namespace GSDK
{
    namespace HAL
    {
        class gSDK_Serial_Manager
        {
        public:
            gSDK_Serial_Manager();
            virtual ~gSDK_Serial_Manager() = 0;

            /*! @note How to use
            *  In order to provide platform crossable gSDK library,
            *  we abstract this class as a serial hardware level.
            *
            *  @note function descriptions:
            *
            *  void start();
            *  @brief After calling this function, gSDK_Serial_Manager should be able to
            *  serial_read and send correctly, through a correct UART part.
            *
            *  void stop();
            *  @brief After calling this function, gSDK_Serial_Manager should close the
            *  serial port.
            *
            *  size_t serial_write(const uint8_t *buf, size_t len);
            *  @brief return sent data length.
            *
            *  size_t serial_read(uint8_t *buf, size_t maxlen);
            *  @brief return serial_read data length.
            *
            *  @attention
            *  when writting and reading data, there might have multi-thread problems.
            *  Abstract class gSDK_Serial_Manager did not consider these issue.
            *  Please be careful when you are going to implement send and serial_read
            *  funtions.
            *
            *  @note
            *  we strongly suggest you to inherit this class in your own file, not just
            *  implement
            *  it in gSDK_Serial_Manager.cpp or inside this class
            *
            * */

            virtual void start() = 0;
            virtual void stop() = 0;

            virtual size_t serial_write(const uint8_t *buf, size_t len) = 0;
            virtual size_t serial_read(uint8_t *buf, size_t maxlen) = 0;

            virtual bool get_device_status() const
            {
                return true;
            }

            static constexpr int BUFF_SIZE = 1024;
        };

    } // HAL
    
} // GSDK

#endif // GSDK_SERIAL_MANAGER_H_



