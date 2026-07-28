#include "mavlink_parameter_client.hpp"

#include <algorithm>
#include <asio/dispatch.hpp>
#include <asio/post.hpp>
#include <cstring>
#include <iostream>

#ifndef GSDK_DebugMsg
#define GSDK_DebugMsg(fmt, args ...) \
    do { printf("\33[39m" fmt "\033[0m\n\r", ##args); } while(0)
#endif


#ifndef GSDK_DebugInfo
#define GSDK_DebugInfo(fmt, args ...) \
    do { printf("\33[34m" fmt "\033[0m\n\r", ##args); } while(0)
#endif


#ifndef GSDK_DebugWarning
#define GSDK_DebugWarning(fmt, args ...) \
    do { printf("\33[33m" fmt "\033[0m\n\r", ##args); } while(0)
#endif


#ifndef GSDK_DebugError
#define GSDK_DebugError(fmt, args ...) \
    do { printf("\33[91m" fmt "\033[0m\n\r", ##args); } while(0)
#endif


#ifndef GSDK_DebugSuccess
#define GSDK_DebugSuccess(fmt, args ...) \
    do { printf("\33[92m" fmt "\033[0m\n\r", ##args); } while(0)
#endif

namespace paramio {

// std::visit helper for std::variant (same idea as MAVSDK's overloaded.hpp).
template<class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template<class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

MavlinkParameterClient::MavlinkParameterClient(
    asio::io_context& io_context,
    TimeoutHandler& timeout_handler,
    TimeoutSCallback timeout_s_callback,
    SendMessageCallback send_message_callback,
    uint8_t own_system_id,
    uint8_t own_component_id,
    uint8_t channel,
    uint8_t target_system_id,
    uint8_t target_component_id) :
    _timeout_handler(timeout_handler),
    _timeout_s_callback(std::move(timeout_s_callback)),
    _send_message_callback(std::move(send_message_callback)),
    _own_system_id(own_system_id),
    _own_component_id(own_component_id),
    _channel(channel),
    _target_system_id(target_system_id),
    _target_component_id(target_component_id),
    _io_context(io_context)
{
}

MavlinkParameterClient::~MavlinkParameterClient() = default;

void MavlinkParameterClient::handle_message(const mavlink_message_t& message)
{
    // If you have several gimbals/targets each using their own client
    // instance, add an extra filter here on message.compid ==
    // _target_component_id (or filter earlier, e.g. in is_gimbal(), as your
    // existing code already does).
    switch (message.msgid) {
        case MAVLINK_MSG_ID_PARAM_VALUE:
            process_param_value(message);
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Set
// ---------------------------------------------------------------------------

MavlinkParameterClient::Result
MavlinkParameterClient::set_param(const std::string& name, const ParamValue& value)
{
    std::promise<Result> prom;
    auto fut = prom.get_future();
    set_param_async(name, value, [&prom](Result r) { prom.set_value(r); }, this);
    return fut.get();
}

void MavlinkParameterClient::set_param_async(
    const std::string& name,
    const ParamValue& value,
    const SetParamCallback& callback,
    const void* cookie)
{

    if (name.size() > PARAM_ID_LEN) {

        GSDK_DebugError("[PARAM SET] Name too long: %s",
                        name.c_str());

        if (callback) callback(Result::ParamNameTooLong);
        return;
    }


    auto work = std::make_shared<WorkItem>(
        WorkItemSet{name,value,callback},
        cookie);


    asio::post(_io_context,[this,work](){

        const bool was_empty = _work_queue.empty();

        _work_queue.push_back(work);

        if(was_empty)
            do_work();

    });
}

MavlinkParameterClient::Result
MavlinkParameterClient::set_param_int(const std::string& name, int32_t value)
{
    std::promise<Result> prom;
    auto fut = prom.get_future();
    set_param_int_async(name, value, [&prom](Result r) { prom.set_value(r); }, this);
    return fut.get();
}

void MavlinkParameterClient::set_param_int_async(
    const std::string& name,
    int32_t value,
    const SetParamCallback& callback,
    const void* cookie)
{

    GSDK_DebugInfo("[SET INT] %s = %ld",
                   name.c_str(),
                   value);


    auto cached = _param_cache.param_by_id(name);


    if(!cached.has_value()){

        GSDK_DebugError("[SET INT] Param not found %s",
                        name.c_str());

        if(callback)
            callback(Result::NotFound);

        return;
    }


    auto value_to_set = cached->value;


    if(!value_to_set.set_int(value)){

        GSDK_DebugError("[SET INT] Wrong type %s",
                        name.c_str());

        if(callback)
            callback(Result::WrongType);

        return;
    }


    if(value_to_set == cached->value){

        GSDK_DebugWarning("[SET INT] Same value skip %s",
                          name.c_str());

        if(callback)
            callback(Result::Success);

        return;
    }


    set_param_async(name,value_to_set,callback,cookie);
}

MavlinkParameterClient::Result
MavlinkParameterClient::set_param_float(const std::string& name, float value)
{
    std::promise<Result> prom;
    auto fut = prom.get_future();
    set_param_float_async(name, value, [&prom](Result r) { prom.set_value(r); }, this);
    return fut.get();
}

void MavlinkParameterClient::set_param_float_async(
    const std::string& name,
    float value,
    const SetParamCallback& callback,
    const void* cookie)
{
    GSDK_DebugInfo("[SET FLOAT] %s = %.3f",
                   name.c_str(),
                   value);


    // Check cache
    auto cached = _param_cache.param_by_id(name);


    if (!cached.has_value()) {

        GSDK_DebugError(
            "[SET FLOAT] Param not found in cache: %s",
            name.c_str());


        if (callback)
            callback(Result::NotFound);

        return;
    }

    // Check type
    if (!cached->value.is<float>()) {

        GSDK_DebugError(
            "[SET FLOAT] Wrong type: %s (cached type is not float)",
            name.c_str());


        if (callback)
            callback(Result::WrongType);

        return;
    }



    float cached_value = cached->value.get<float>();


    GSDK_DebugInfo(
        "[SET FLOAT] Current=%.3f New=%.3f",
        cached_value,
        value);



    // Same value -> skip
    if (cached_value == value) {

        GSDK_DebugWarning(
            "[SET FLOAT] Value unchanged, skip sending: %s",
            name.c_str());


        if (callback)
            callback(Result::Success);

        return;
    }

    ParamValue value_to_set;

    value_to_set.set<float>(value);



    set_param_async(
        name,
        value_to_set,
        callback,
        cookie);
}

// ---------------------------------------------------------------------------
// Get
// ---------------------------------------------------------------------------

std::pair<MavlinkParameterClient::Result, ParamValue>
MavlinkParameterClient::get_param(const std::string& name)
{

    std::promise<std::pair<Result, ParamValue>> prom;
    auto fut = prom.get_future();

    get_param_async(
        name,
        [&prom, name](Result r, ParamValue v) {
            GSDK_DebugInfo("[GET] Callback: %s result=%d value=%s",
                           name.c_str(),
                           static_cast<int>(r),
                           v.get_string().c_str());

            prom.set_value({r, std::move(v)});
        },
        this);

    auto result = fut.get();

    return result;
}

void MavlinkParameterClient::get_param_async(
    const std::string& name,
    const GetParamAnyCallback& callback,
    const void* cookie)
{
    if (name.size() > PARAM_ID_LEN) {

        GSDK_DebugError("[GET ASYNC] Param name too long: %s",
                        name.c_str());

        if (callback) callback(Result::ParamNameTooLong, {});
        return;
    }

    auto work = std::make_shared<WorkItem>(
        WorkItemGet{std::variant<std::string, int16_t>(name), callback},
        cookie);

    asio::post(_io_context, [this, work]() {

        const bool was_empty = _work_queue.empty();

        _work_queue.push_back(work);

        if (was_empty) {
            do_work();
        }
    });
}

std::pair<MavlinkParameterClient::Result, int32_t>
MavlinkParameterClient::get_param_int(const std::string& name)
{

    std::promise<std::pair<Result, int32_t>> prom;
    auto fut = prom.get_future();

    get_param_int_async(
        name,
        [&prom, name](Result r, int32_t v) {

            prom.set_value({r, v});
        },
        this);

    return fut.get();
}

void MavlinkParameterClient::get_param_int_async(
    const std::string& name,
    const std::function<void(Result, int32_t)>& callback,
    const void* cookie)
{

    get_param_async(
        name,
        [callback, name](Result r, ParamValue v)
        {
            if (r != Result::Success) {

                GSDK_DebugError("[GET INT] %s failed result=%d",
                                name.c_str(),
                                static_cast<int>(r));

                if (callback) callback(r, 0);
                return;
            }

            int32_t out = 0;
            const bool ok = extract_int32(v, out);

            GSDK_DebugInfo("[GET INT] %s extract=%s value=%ld",
                           name.c_str(),
                           ok ? "OK" : "FAIL",
                           (long)out);

            if (callback)
                callback(ok ? Result::Success : Result::WrongType, out);
        },
        cookie);
}

std::pair<MavlinkParameterClient::Result, float>
MavlinkParameterClient::get_param_float(const std::string& name)
{

    std::promise<std::pair<Result, float>> prom;
    auto fut = prom.get_future();

    get_param_float_async(
        name,
        [&prom, name](Result r, float v)
        {
            prom.set_value({r, v});
        },
        this);

    return fut.get();
}

void MavlinkParameterClient::get_param_float_async(
    const std::string& name,
    const std::function<void(Result, float)>& callback,
    const void* cookie)
{

    get_param_async(
        name,
        [callback, name](Result r, ParamValue v)
        {
            if (r == Result::Success && v.is<float>()) {

                float value = v.get<float>();

                if (callback)
                    callback(Result::Success, value);

            } else {

                if (r == Result::Success) {

                    GSDK_DebugError("[GET FLOAT] Wrong type: %s",
                                    name.c_str());

                } else {

                    GSDK_DebugError("[GET FLOAT] Failed: %s result=%d",
                                    name.c_str(),
                                    static_cast<int>(r));
                }

                if (callback)
                    callback(r == Result::Success ? Result::WrongType : r,
                             0.0f);
            }
        },
        cookie);
}

// ---------------------------------------------------------------------------
// Get all
// ---------------------------------------------------------------------------

std::pair<MavlinkParameterClient::Result, std::map<std::string, ParamValue>>
MavlinkParameterClient::get_all_params()
{
    std::promise<std::pair<Result, std::map<std::string, ParamValue>>> prom;
    auto fut = prom.get_future();
    get_all_params_async(
        [&prom](Result r, std::map<std::string, ParamValue> m) { prom.set_value({r, std::move(m)}); },
        this);
    return fut.get();
}

void MavlinkParameterClient::get_all_params_async(GetAllParamsCallback callback, const void* cookie)
{
    _param_list_state = ParamListState::Requesting;
 
    // Wrap the user's callback so _param_list_state is kept up to date no
    // matter which code path ends up calling it (send failure in do_work(),
    // success in process_param_value(), or timeout in receive_timeout()) -
    // they all funnel through this single WorkItemGetAll::callback.
    GetAllParamsCallback wrapped_callback =
        [this, callback](Result result, std::map<std::string, ParamValue> all_params) {
            _param_list_state =
                (result == Result::Success) ? ParamListState::Received : ParamListState::Failed;
            if (callback) callback(result, std::move(all_params));
        };
 
    auto work =
        std::make_shared<WorkItem>(WorkItemGetAll{std::move(wrapped_callback), 0, false}, cookie);
    asio::post(_io_context, [this, work]() {
        const bool was_empty = _work_queue.empty();
        _work_queue.push_back(work);
        if (was_empty) do_work();
    });
}
// ---------------------------------------------------------------------------
// Direct cache reads (no network)
// ---------------------------------------------------------------------------

bool MavlinkParameterClient::has_cached_param(const std::string& name) const
{
    return _param_cache.exists(name);
}

std::optional<ParamValue> MavlinkParameterClient::get_cached_param(const std::string& name) const
{

    auto p = _param_cache.param_by_id(name);

    if (!p.has_value()) {
        GSDK_DebugWarning("[CACHE] %s not found", name.c_str());
        return std::nullopt;
    }

    return p->value;
}

std::optional<int32_t> MavlinkParameterClient::get_cached_param_int(const std::string& name) const
{

    auto v = get_cached_param(name);

    if (!v.has_value()) {
        return std::nullopt;
    }

    int32_t out = 0;

    if (!extract_int32(*v, out)) {
        GSDK_DebugError("[CACHE INT] %s wrong type (%s)",
                        name.c_str(),
                        v->get_string().c_str());
        return std::nullopt;
    }


    return out;
}

std::optional<float> MavlinkParameterClient::get_cached_param_float(const std::string& name) const
{

    auto v = get_cached_param(name);

    if (!v.has_value()) {
        return std::nullopt;
    }

    if (!v->is<float>()) {
        GSDK_DebugError("[CACHE FLOAT] %s wrong type (%s)",
                        name.c_str(),
                        v->get_string().c_str());
        return std::nullopt;
    }

    float value = v->get<float>();


    return value;
}

bool MavlinkParameterClient::extract_int32(const ParamValue& value, int32_t& out)
{
    if (value.is<int32_t>()) out = value.get<int32_t>();
    else if (value.is<uint32_t>()) out = static_cast<int32_t>(value.get<uint32_t>());
    else if (value.is<int16_t>()) out = value.get<int16_t>();
    else if (value.is<uint16_t>()) out = value.get<uint16_t>();
    else if (value.is<int8_t>()) out = value.get<int8_t>();
    else if (value.is<uint8_t>()) out = value.get<uint8_t>();
    else return false;
    return true;
}

// ---------------------------------------------------------------------------
// Subscribe (pub/sub on param changes)
// ---------------------------------------------------------------------------

void MavlinkParameterClient::subscribe_param_changed(
    const std::string& name, const ParamChangedCallback& callback, const void* cookie)
{
    asio::post(_io_context, [this, name, callback, cookie]() {
        _subscriptions.push_back({name, callback, cookie});
    });
}

void MavlinkParameterClient::unsubscribe_param_changed(const std::string& name, const void* cookie)
{
    asio::post(_io_context, [this, name, cookie]() {
        _subscriptions.erase(
            std::remove_if(
                _subscriptions.begin(),
                _subscriptions.end(),
                [&](const auto& s) { return s.cookie == cookie && s.param_name == name; }),
            _subscriptions.end());
    });
}

void MavlinkParameterClient::unsubscribe_all_params_changed(const void* cookie)
{
    _subscriptions.erase(
        std::remove_if(
            _subscriptions.begin(),
            _subscriptions.end(),
            [&](const auto& s) { return s.cookie == cookie; }),
        _subscriptions.end());
}

void MavlinkParameterClient::find_and_call_subscriptions_value_changed(
    const std::string& id, const ParamValue& value)
{
    for (const auto& sub : _subscriptions) {
        if (sub.param_name.empty() || sub.param_name == id) {
            if (sub.callback) sub.callback(id, value);
        }
    }
}

// ---------------------------------------------------------------------------
// Cancel / clear
// ---------------------------------------------------------------------------

void MavlinkParameterClient::cancel_all_param(const void* cookie)
{
    if (_io_context.stopped()) {
        _work_queue.erase(
            std::remove_if(
                _work_queue.begin(), _work_queue.end(),
                [&](auto&& item) { return item->cookie == cookie; }),
            _work_queue.end());
        unsubscribe_all_params_changed(cookie);
        return;
    }
    std::promise<void> done;
    asio::dispatch(_io_context, [this, cookie, &done]() {
        _work_queue.erase(
            std::remove_if(
                _work_queue.begin(), _work_queue.end(),
                [&](auto&& item) { return item->cookie == cookie; }),
            _work_queue.end());
        unsubscribe_all_params_changed(cookie);
        done.set_value();
    });
    done.get_future().wait();
}

void MavlinkParameterClient::clear_cache()
{
    _param_cache.clear();
}

// ---------------------------------------------------------------------------
// Work queue engine
// ---------------------------------------------------------------------------

void MavlinkParameterClient::do_work()
{
    if (_work_queue.empty()) {
        GSDK_DebugWarning("[PARAM_WORK] Queue empty");
        return;
    }


    auto work = _work_queue.front();


    if (work->already_requested) {

        GSDK_DebugWarning(
            "[PARAM_WORK] Already requested, waiting PARAM_VALUE"
        );

        return;
    }

    std::visit(
        overloaded{


            [&](WorkItemSet& item) {



                if (!send_set_param_message(item)) {


                    GSDK_DebugError(
                        "[PARAM_WORK][SET] Send failed param=%s",
                        item.param_name.c_str()
                    );


                    _work_queue.pop_front();


                    if (!_work_queue.empty())
                        asio::post(
                            _io_context,
                            [this] { do_work(); }
                        );


                    if (item.callback)
                        item.callback(
                            Result::ConnectionError
                        );


                    return;
                }


                work->already_requested = true;



                _timeout_cookie =
                    _timeout_handler.add(
                        [this] { receive_timeout(); },
                        _timeout_s_callback()
                    );

            },


            [&](WorkItemGet& item) {

                if (std::holds_alternative<std::string>(
                        item.param_identifier)) {


                } else {


                    GSDK_DebugInfo(
                        "[PARAM_WORK][GET] index=%d",
                        std::get<int16_t>(
                            item.param_identifier
                        )
                    );
                }



                if (!send_get_param_message(item)) {


                    GSDK_DebugError(
                        "[PARAM_WORK][GET] Send failed"
                    );


                    _work_queue.pop_front();


                    if (!_work_queue.empty())
                        asio::post(
                            _io_context,
                            [this] { do_work(); }
                        );


                    if (item.callback)
                        item.callback(
                            Result::ConnectionError,
                            {}
                        );


                    return;
                }


                work->already_requested = true;



                _timeout_cookie =
                    _timeout_handler.add(
                        [this] { receive_timeout(); },
                        _timeout_s_callback()
                    );



            },


            [&](WorkItemGetAll& item) {


                clear_cache();



                if (!send_request_list_message()) {


                    GSDK_DebugError(
                        "[PARAM_WORK][GET_ALL] Send request list failed"
                    );


                    _work_queue.pop_front();


                    if (!_work_queue.empty())
                        asio::post(
                            _io_context,
                            [this] { do_work(); }
                        );


                    if (item.callback)
                        item.callback(
                            Result::ConnectionError,
                            {}
                        );


                    return;
                }


                work->already_requested = true;



                _timeout_cookie =
                    _timeout_handler.add(
                        [this] { receive_timeout(); },
                        _timeout_s_callback() *
                        _get_all_timeout_factor
                    );

            }

        },
        work->work_item_variant
    );
}

bool MavlinkParameterClient::send_set_param_message(WorkItemSet& item)
{
    auto param_id = param_id_to_buffer(item.param_name);
    const float value_set = item.param_value.get_4_float_bytes();

    mavlink_message_t message;
    mavlink_msg_param_set_pack_chan(
        _own_system_id,
        _own_component_id,
        _channel,
        &message,
        _target_system_id,
        _target_component_id,
        param_id.data(),
        value_set,
        item.param_value.get_mav_param_type());
    return _send_message_callback(message);
}

bool MavlinkParameterClient::send_get_param_message(WorkItemGet& item)
{
    std::array<char, PARAM_ID_LEN> buf{};
    int16_t index = -1;
    if (auto str = std::get_if<std::string>(&item.param_identifier)) {
        buf = param_id_to_buffer(*str);
    } else {
        index = std::get<int16_t>(item.param_identifier);
    }
    return send_get_param_message(buf, index);
}

bool MavlinkParameterClient::send_get_param_message(
    const std::array<char, PARAM_ID_LEN>& id_buf, int16_t index)
{
    mavlink_message_t message;
    mavlink_msg_param_request_read_pack_chan(
        _own_system_id,
        _own_component_id,
        _channel,
        &message,
        _target_system_id,
        _target_component_id,
        id_buf.data(),
        index);
    return _send_message_callback(message);
}

bool MavlinkParameterClient::send_request_list_message()
{
    mavlink_message_t message;
    mavlink_msg_param_request_list_pack_chan(
        _own_system_id, _own_component_id, _channel, &message, _target_system_id,
        _target_component_id);
    return _send_message_callback(message);
}

bool MavlinkParameterClient::request_next_missing(uint16_t count)
{
    const uint16_t chunk_size = 10;
    auto missing = _param_cache.next_missing_indices(count, chunk_size);
    if (missing.empty()) return false;

    for (auto idx : missing) {
        std::array<char, PARAM_ID_LEN> empty_id{};
        if (!send_get_param_message(empty_id, static_cast<int16_t>(idx))) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Receiving PARAM_VALUE
// ---------------------------------------------------------------------------

void MavlinkParameterClient::process_param_value(const mavlink_message_t& message)
{
    mavlink_param_value_t param_value;
    mavlink_msg_param_value_decode(&message, &param_value);


    const std::string id = extract_safe_param_id(param_value.param_id);


    if (id.empty()) {
        GSDK_DebugError("[PARAM_VALUE] Empty parameter ID");
        return;
    }


    ParamValue received;

    if (!received.set_from_mavlink_param_value(param_value)) {

        GSDK_DebugError(
            "[PARAM_VALUE] Failed decode param=%s type=%f",
            id.c_str()
        );

        return;
    }

    if (_work_queue.empty()) {

        GSDK_DebugInfo(
            "[%s] No pending request -> cache update value = %f",
            id.c_str(), 
            param_value.param_value
        );


        _param_cache.update_existing_param(id, received);

        find_and_call_subscriptions_value_changed(
            id,
            received
        );

        return;
    }



    auto work = _work_queue.front();


    if (!work->already_requested) {

        GSDK_DebugWarning(
            "[PARAM_VALUE] Work exists but not requested yet"
        );

        return;
    }



    std::visit(
        overloaded{


            [&](WorkItemSet& item) {

                GSDK_DebugInfo(
                    "[PARAM_SET] Expect=%s RX=%s",
                    item.param_name.c_str(),
                    id.c_str()
                );


                if (item.param_name != id) {

                    GSDK_DebugWarning(
                        "[PARAM_SET] ID mismatch expect=%s got=%s",
                        item.param_name.c_str(),
                        id.c_str()
                    );

                    return;
                }



                GSDK_DebugInfo(
                    "[PARAM_SET] value expect=%s receive=%s",
                    item.param_value.get_string().c_str(),
                    received.get_string().c_str()
                );



                if (!item.param_value.is_same_type(received)) {

                    GSDK_DebugError(
                        "[PARAM_SET] Wrong type param=%s",
                        id.c_str()
                    );


                    _timeout_handler.remove(_timeout_cookie);

                    _work_queue.pop_front();


                    if (!_work_queue.empty()) {
                        asio::post(
                            _io_context,
                            [this] { do_work(); }
                        );
                    }


                    if (item.callback) {
                        item.callback(Result::WrongType);
                    }

                    return;
                }




                if (item.param_value == received) {


                    GSDK_DebugSuccess(
                        "[PARAM_SET] SUCCESS %s=%s",
                        id.c_str(),
                        received.get_string().c_str()
                    );


                    _timeout_handler.remove(_timeout_cookie);


                    _param_cache.update_existing_param(
                        item.param_name,
                        item.param_value
                    );


                    _work_queue.pop_front();


                    if (!_work_queue.empty()) {
                        asio::post(
                            _io_context,
                            [this] { do_work(); }
                        );
                    }


                    if (item.callback) {
                        item.callback(Result::Success);
                    }


                } 
                else if (work->retries_to_do > 0) {


                    GSDK_DebugWarning(
                        "[PARAM_SET] Mismatch retry=%d",
                        work->retries_to_do
                    );


                    if (!send_set_param_message(item)) {


                        GSDK_DebugError(
                            "[PARAM_SET] resend failed"
                        );


                        _work_queue.pop_front();


                        if (!_work_queue.empty()) {
                            asio::post(
                                _io_context,
                                [this] { do_work(); }
                            );
                        }


                        if (item.callback) {
                            item.callback(Result::ConnectionError);
                        }


                    } 
                    else {

                        --work->retries_to_do;

                        _timeout_handler.refresh(
                            _timeout_cookie
                        );
                    }

                } 
                else {


                    GSDK_DebugError(
                        "[PARAM_SET] Timeout %s",
                        id.c_str()
                    );


                    _work_queue.pop_front();


                    if (!_work_queue.empty()) {
                        asio::post(
                            _io_context,
                            [this] { do_work(); }
                        );
                    }


                    if (item.callback) {
                        item.callback(Result::Timeout);
                    }
                }

            },




            [&](WorkItemGet& item) {

                if (!validate_id_or_index(
                        item.param_identifier,
                        id,
                        static_cast<int16_t>(param_value.param_index))) {


                    GSDK_DebugWarning(
                        "[PARAM_GET] Invalid ID/index %s",
                        id.c_str()
                    );

                    return;
                }



                GSDK_DebugSuccess(
                    "[PARAM_GET] SUCCESS %s=%s",
                    id.c_str(),
                    received.get_string().c_str()
                );



                _timeout_handler.remove(
                    _timeout_cookie
                );



                if (_param_cache.update_existing_param(
                        id,
                        received)
                    == MavlinkParameterCache::UpdateExistingParamResult::MissingParam) {


                    _param_cache.add_new_param(
                        id,
                        received,
                        param_value.param_index
                    );
                }



                _work_queue.pop_front();



                if (!_work_queue.empty()) {
                    asio::post(
                        _io_context,
                        [this] { do_work(); }
                    );
                }



                if (item.callback) {
                    item.callback(
                        Result::Success,
                        received
                    );
                }

            },





            [&](WorkItemGetAll& item) {

                GSDK_DebugInfo(
                    "[PARAM_GET_ALL] %-16s | %3d/%-3d | %10.3f",
                    id.c_str(),
                    param_value.param_index,
                    param_value.param_count - 1,
                    param_value.param_value
                );

                auto maybe_last_missing =
                    _param_cache.last_missing_requested();



                auto add_result =
                    _param_cache.add_new_param(
                        id,
                        received,
                        param_value.param_index
                    );



                if (add_result ==
                    MavlinkParameterCache::AddNewParamResult::TooManyParams) {


                    GSDK_DebugError(
                        "[PARAM_GET_ALL] Too many params"
                    );

                    return;
                }



                item.count = param_value.param_count;



                if (_param_cache.count() ==
                    param_value.param_count) {


                    GSDK_DebugSuccess(
                        "[PARAM_GET_ALL] COMPLETE count=%d",
                        item.count
                    );



                    _timeout_handler.remove(
                        _timeout_cookie
                    );


                    _work_queue.pop_front();



                    if (!_work_queue.empty()) {
                        asio::post(
                            _io_context,
                            [this] { do_work(); }
                        );
                    }



                    if (item.callback) {
                        item.callback(
                            Result::Success,
                            _param_cache.all_parameters_map()
                        );
                    }


                    return;
                }



                if (item.rerequesting &&
                    maybe_last_missing == param_value.param_index) {


                    GSDK_DebugWarning(
                        "[PARAM_GET_ALL] Missing param retry"
                    );



                    if (!request_next_missing(item.count)) {


                        _work_queue.pop_front();


                        if (!_work_queue.empty()) {
                            asio::post(
                                _io_context,
                                [this] { do_work(); }
                            );
                        }


                        if (item.callback) {
                            item.callback(
                                Result::ConnectionError,
                                {}
                            );
                        }

                        return;
                    }
                }



                _timeout_handler.refresh(
                    _timeout_cookie
                );
            }

        },
        work->work_item_variant
    );
}

void MavlinkParameterClient::receive_timeout()
{
    if (_work_queue.empty()) {
        GSDK_DebugWarning("[PARAM_TIMEOUT] Work queue empty");
        return;
    }


    auto work = _work_queue.front();


    if (!work->already_requested) {
        GSDK_DebugWarning("[PARAM_TIMEOUT] Work not requested yet");
        return;
    }



    std::visit(
        overloaded{


            [&](WorkItemSet& item) {

                GSDK_DebugWarning(
                    "[PARAM_TIMEOUT][SET] param=%s retry=%d",
                    item.param_name.c_str(),
                    work->retries_to_do
                );


                if (work->retries_to_do > 0) {


                    GSDK_DebugInfo(
                        "[PARAM_TIMEOUT][SET] Resend SET param=%s",
                        item.param_name.c_str()
                    );


                    if (!send_set_param_message(item)) {


                        GSDK_DebugError(
                            "[PARAM_TIMEOUT][SET] Send failed param=%s",
                            item.param_name.c_str()
                        );


                        _work_queue.pop_front();


                        if (!_work_queue.empty())
                            asio::post(_io_context, [this] { do_work(); });


                        if (item.callback)
                            item.callback(Result::ConnectionError);


                    } else {

                        --work->retries_to_do;

                        _timeout_cookie =
                            _timeout_handler.add(
                                [this] { receive_timeout(); },
                                _timeout_s_callback()
                            );
                    }


                } else {


                    GSDK_DebugError(
                        "[PARAM_TIMEOUT][SET] Timeout final param=%s",
                        item.param_name.c_str()
                    );


                    _work_queue.pop_front();


                    if (!_work_queue.empty())
                        asio::post(_io_context, [this] { do_work(); });


                    if (item.callback)
                        item.callback(Result::Timeout);
                }
            },




            [&](WorkItemGet& item) {


                GSDK_DebugWarning(
                    "[PARAM_TIMEOUT][GET] retry=%d",
                    work->retries_to_do
                );


                if (work->retries_to_do > 0) {


                    GSDK_DebugInfo(
                        "[PARAM_TIMEOUT][GET] Resend GET"
                    );


                    if (!send_get_param_message(item)) {


                        GSDK_DebugError(
                            "[PARAM_TIMEOUT][GET] Send failed"
                        );


                        _work_queue.pop_front();


                        if (!_work_queue.empty())
                            asio::post(_io_context, [this] { do_work(); });


                        if (item.callback)
                            item.callback(Result::ConnectionError, {});


                    } else {


                        --work->retries_to_do;


                        GSDK_DebugInfo(
                            "[PARAM_TIMEOUT][GET] Retry left=%d",
                            work->retries_to_do
                        );


                        _timeout_cookie =
                            _timeout_handler.add(
                                [this] { receive_timeout(); },
                                _timeout_s_callback()
                            );
                    }


                } else {


                    GSDK_DebugError(
                        "[PARAM_TIMEOUT][GET] Timeout final"
                    );


                    _work_queue.pop_front();


                    if (!_work_queue.empty())
                        asio::post(_io_context, [this] { do_work(); });


                    if (item.callback)
                        item.callback(Result::Timeout, {});
                }
            },





            [&](WorkItemGetAll& item) {


                GSDK_DebugWarning(
                    "[PARAM_TIMEOUT][GET_ALL] count=%d retry=%d",
                    item.count,
                    work->retries_to_do
                );



                if (item.count == 0) {


                    GSDK_DebugWarning(
                        "[PARAM_TIMEOUT][GET_ALL] No parameter received, resend request list"
                    );



                    if (work->retries_to_do > 0) {


                        --work->retries_to_do;


                        if (!send_request_list_message()) {


                            GSDK_DebugError(
                                "[PARAM_TIMEOUT][GET_ALL] Request list failed"
                            );


                            _work_queue.pop_front();


                            if (!_work_queue.empty())
                                asio::post(_io_context, [this] { do_work(); });


                            if (item.callback)
                                item.callback(Result::ConnectionError, {});


                            return;
                        }



                        GSDK_DebugInfo(
                            "[PARAM_TIMEOUT][GET_ALL] Request list retry left=%d",
                            work->retries_to_do
                        );



                        _timeout_cookie =
                            _timeout_handler.add(
                                [this] { receive_timeout(); },
                                _timeout_s_callback() *
                                _get_all_timeout_factor
                            );


                    } else {


                        GSDK_DebugError(
                            "[PARAM_TIMEOUT][GET_ALL] Timeout no parameter received"
                        );


                        _work_queue.pop_front();


                        if (!_work_queue.empty())
                            asio::post(_io_context, [this] { do_work(); });


                        if (item.callback)
                            item.callback(Result::Timeout, {});
                    }


                    return;
                }



                GSDK_DebugWarning(
                    "[PARAM_TIMEOUT][GET_ALL] Partial received=%d, request missing",
                    item.count
                );


                item.rerequesting = true;


                if (!request_next_missing(item.count)) {


                    GSDK_DebugError(
                        "[PARAM_TIMEOUT][GET_ALL] Request missing failed"
                    );


                    _work_queue.pop_front();


                    if (!_work_queue.empty())
                        asio::post(_io_context, [this] { do_work(); });


                    if (item.callback)
                        item.callback(Result::ConnectionError, {});


                    return;
                }



                GSDK_DebugInfo(
                    "[PARAM_TIMEOUT][GET_ALL] Missing request sent"
                );


                _timeout_cookie =
                    _timeout_handler.add(
                        [this] { receive_timeout(); },
                        _timeout_s_callback()
                    );
            }


        },
        work->work_item_variant
    );
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool MavlinkParameterClient::validate_id_or_index(
    const std::variant<std::string, int16_t>& original, const std::string& id, int16_t index)
{
    if (auto str = std::get_if<std::string>(&original)) {
        return id == *str;
    }
    return std::get<int16_t>(original) == index;
}

std::array<char, MavlinkParameterClient::PARAM_ID_LEN>
MavlinkParameterClient::param_id_to_buffer(const std::string& name)
{
    std::array<char, PARAM_ID_LEN> buf{};
    std::memcpy(buf.data(), name.c_str(), std::min(name.size(), PARAM_ID_LEN));
    return buf;
}

std::string MavlinkParameterClient::extract_safe_param_id(const char* raw)
{
    // MAVLink's param_id isn't guaranteed to be null-terminated if it's
    // exactly 16 characters long -> bound it manually.
    return std::string(raw, strnlen(raw, PARAM_ID_LEN));
}

} // namespace paramio