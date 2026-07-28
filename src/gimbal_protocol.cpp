/*******************************************************************************
 * @file    gimbal_protocol.cpp
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-21-2018
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

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <errno.h>

#include "gimbal_protocol.h"

Gimbal_Protocol::Gimbal_Protocol(Generic_Port *port,
                                 const mavlink_system_t &system, 
                                 mavlink_channel_t channel) :
    _system(system),
    _attitude()
{
    _port = port;
    _channel     = channel;
    pthread_mutex_init(&_mutex, NULL);
    pthread_cond_init(&_condition, NULL);
}

Gimbal_Protocol::~Gimbal_Protocol()
{
    pthread_mutex_destroy(&_mutex);
    pthread_cond_destroy(&_condition);
}

/**
 * @brief Function init protocol
 *
 * @param gimbal
 */
void Gimbal_Protocol::initialize(const mavlink_system_t &gimbal)
{
    _gimbal.sysid  = gimbal.sysid;
    _gimbal.compid = gimbal.compid;

    _param_client.emplace(
        _param_io_context,
        _param_timeout_handler,
        [] { return 0.5; },   // time out in seconds
        [this](const mavlink_message_t& msg) { return _port->write_message(msg) > 0; },
        _system.sysid,        // own_system_id
        _system.compid,       // own_component_id
        _channel,              // channel
        _gimbal.sysid,         // target_system_id
        _gimbal.compid);       // target_component_id

    _param_io_thread = std::thread([this] { _param_io_context.run(); });

    _is_init = true;

}

/**
 * @brief Function send commad long to gimbal
 *
 * @param command command ID
 * @param param param1 -> param7
 * @return result_t SUCCESS if send command successfully
 */
Gimbal_Protocol::result_t Gimbal_Protocol::send_command_long(uint16_t command, const float param[7])
{
    if (_port == nullptr) {
        GSDK_DebugError("ERROR: serial port not exist\n");
        throw 1;
    }

    if (!_is_init) {
        GSDK_DebugError("ERROR: could not send COMMAND_LONG, gimbal proto has not been initialized\n");
        return ERROR;
    }

    // Prepare command for off-board mode
    mavlink_command_long_t cmd = { 0 };
    cmd.target_system    = _gimbal.sysid;
    cmd.target_component = _gimbal.compid;
    cmd.command          = command;
    memcpy(&cmd.param1, param, 7 * sizeof(float));
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = { 0 };
    // mavlink_msg_command_long_encode(_system.sysid, _system.compid, &message, &cmd);
    mavlink_msg_command_long_encode_chan(_system.sysid, _system.compid, _channel, &message, &cmd);
    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    return (_port->write_message(message) > 0) ? SUCCESS : ERROR;
}

/**
 * @brief Function send commad long to gimbal and block to wait for ack response
 *
 * @param command command ID
 * @param param param1 -> param7
 * @return result_t ack response from gimbal
 */
Gimbal_Protocol::result_t Gimbal_Protocol::send_command_long_sync(uint16_t command, const float param[7])
{

    auto request = std::make_shared<PendingRequest>();

    request->command = command;

    auto future = request->promise.get_future();

    {
        std::lock_guard<std::mutex> lock(_pending_mutex);
        _pending_requests[command] = request;
    }

    if (send_command_long(command, param) != SUCCESS)
    {
        std::lock_guard<std::mutex> lock(_pending_mutex);
        _pending_requests.erase(command);
        return UNKNOWN;
    }

    auto status = future.wait_for(std::chrono::seconds(1));

    if (status == std::future_status::timeout)
    {
        std::lock_guard<std::mutex> lock(_pending_mutex);
        _pending_requests.erase(command);
        return TIMEOUT;
    }

    return future.get();
}

/**
 * @brief callback for command ack
 * @details called from serial read message
 * @param message
 */
void Gimbal_Protocol::command_ack_callback(const mavlink_message_t &message)
{
    if (!_is_init) {
        return;
    }

    // Reset ack
    memset(&_ack, 0, sizeof(mavlink_command_ack_t));
    mavlink_msg_command_ack_decode(&message, &_ack);

    if (_ack.target_system != _system.sysid ||
        _ack.target_component != _system.compid)
    {
        return;
    }

    std::shared_ptr<PendingRequest> request;

    {
        std::lock_guard<std::mutex> lock(_pending_mutex);

        auto it = _pending_requests.find(_ack.command);

        if (it == _pending_requests.end())
            return;

        request = it->second;

        _pending_requests.erase(it);
    }

    request->promise.set_value(
        from_mav_result((MAV_RESULT)_ack.result));
}

/**
 * @brief convert from mav result
 *
 * @param res
 * @return result_t
 */
Gimbal_Protocol::result_t Gimbal_Protocol::from_mav_result(MAV_RESULT res)
{
    switch (res) {
        case MAV_RESULT_ACCEPTED:
            return SUCCESS;
            break;

        case MAV_RESULT_DENIED:
            return DENIED;
            break;

        case MAV_RESULT_IN_PROGRESS:
            return SUCCESS;
            break;

        default:
            return UNKNOWN;
            break;
    }

    return UNKNOWN;
}

/**
 * @brief Update gimbal attitude for gimbal protocol instance
 *
 * @param pitch
 * @param roll
 * @param yaw
 */
void Gimbal_Protocol::update_attitude(float pitch, float roll, float yaw)
{
    _attitude.pitch = pitch;
    _attitude.roll  = roll;
    _attitude.yaw   = yaw;
}

/**
 * @brief Update gimbal attitude for gimbal protocol instance
 *
 * @param q attitude quaternion
 */
void Gimbal_Protocol::update_attitude(const float q[4])
{
    mavlink_quaternion_to_euler(q, &_attitude.roll, &_attitude.pitch, &_attitude.yaw);
    (void)_attitude.to_deg();
}


void Gimbal_Protocol::get_attitude(float &pitch, float &roll, float &yaw){
    pitch = _attitude.pitch;
    roll = _attitude.roll;
    yaw = _attitude.yaw;
}

bool Gimbal_Protocol::has_param(const std::string& name) 
{
    if (!_param_client.has_value()) return false;
    return _param_client->has_cached_param(name);
}

void Gimbal_Protocol::request_all_params()
{
    if (!_param_client.has_value()) return;
    _param_client->get_all_params_async(
        [this](paramio::MavlinkParameterClient::Result result,
        std::map<std::string, paramio::ParamValue>) {
            if (result == paramio::MavlinkParameterClient::Result::Success) {
                GSDK_DebugSuccess("Fetched full gimbal param list");
            } else {
                GSDK_DebugError("Failed to fetch full gimbal param list");
            }
        },this);
}
void Gimbal_Protocol::request_all_params(paramio::MavlinkParameterClient::GetAllParamsCallback callback)
{
    if (!_param_client.has_value()) return;
    _param_client->get_all_params_async(std::move(callback),this);
}

paramio::MavlinkParameterClient::Result
Gimbal_Protocol::set_param_float(const std::string& name, float value)
{
    if (!_param_client.has_value()) {
        return paramio::MavlinkParameterClient::Result::ConnectionError;
    }

    return _param_client->set_param_float(name, value);
}

void Gimbal_Protocol::set_param_float_async(const std::string& name, float value, const paramio::MavlinkParameterClient::SetParamCallback& callback)
{
    if (!_param_client.has_value()) return;
    _param_client->set_param_float_async(name, value, callback, this);
}

std::optional<float> Gimbal_Protocol::get_param_float(const std::string& name) 
{
    if (!_param_client.has_value()) return std::nullopt;
    return _param_client->get_cached_param_float(name);
}

void Gimbal_Protocol::request_param_float_async(
        const std::string& name, const std::function<void(paramio::MavlinkParameterClient::Result, float)>& callback)
{
    if (!_param_client.has_value()) return;
    _param_client->get_param_float_async(name, callback, this);
}

std::pair<paramio::MavlinkParameterClient::Result, float>
Gimbal_Protocol::request_param_float(const std::string& name)
{
    if (!_param_client.has_value()) {
        return {paramio::MavlinkParameterClient::Result::ConnectionError, 0.0f};
    }
    // Blocking - sends PARAM_REQUEST_READ, waits for response/timeout.
    // The cache is updated internally by the module regardless of whether
    // the value is used here or not.
    return _param_client->get_param_float(name);
}

void Gimbal_Protocol::param_value_callback(const mavlink_message_t &message)
{
    if (_param_client.has_value()) {
        _param_client->handle_message(message);
    }
}

paramio::MavlinkParameterClient::ParamListState Gimbal_Protocol::get_param_list_state() const
{
    return _param_client.has_value()
        ? _param_client->get_param_list_state()
        : paramio::MavlinkParameterClient::ParamListState::NotRequested;
}