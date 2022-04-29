/*******************************************************************************
 * @file    gimbal_protocol_v1.h
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

#ifndef GIMBAL_PROTOCOL_V1_H_
#define GIMBAL_PROTOCOL_V1_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "gimbal_protocol.h"

// ----------------------------------------------------------------------------------
//   Gimbal Protocol V1 Class
// ----------------------------------------------------------------------------------
/*
 * Gimbal Protocol Class V1
 *
 * This class implement class for Gimbal Protocol V1
 */
class Gimbal_Protocol_V1 : public Gimbal_Protocol
{
public:
    Gimbal_Protocol_V1(Serial_Port *serial_port,
                       const mavlink_system_t &system);
    ~Gimbal_Protocol_V1() = default;

    /**
     * @brief  This function set gimbal mode
     * @param: type see control_mode_t
     * @ret: result
     */
    result_t set_gimbal_mode_sync(control_mode_t mode) override;

    /**
     * @brief  This function reset gimbal with some mode
     * @param: type see gimbal_reset_mode_t
     * @ret: result
     */
    result_t set_gimbal_reset_mode(gimbal_reset_mode_t reset_mode) override;

    /**
     * @brief Set the gimbal move sync
     *
     * @param pitch control pitch value
     * @param roll control roll value
     * @param yaw control yaw value
     * @param mode see input_mode_t
     * @return result_t
     */
    result_t set_gimbal_move_sync(float pitch, float roll, float yaw,
                                  input_mode_t mode) override;

private:

};

#endif // GIMBAL_PROTOCOL_V1_H_
