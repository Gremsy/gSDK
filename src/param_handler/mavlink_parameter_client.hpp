#pragma once

#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "mavlink_parameter_cache.hpp"
#include "param_value.hpp"
#include "timeout_handler.hpp"
#include <asio/io_context.hpp>
#include <common/mavlink.h>

namespace paramio {

// MAVLink parameter management module: get/set (sync + async), fetch all
// params, a cache that tracks missing params, automatic timeout + retry on
// packet loss, and subscribe to be notified when a param's value changes
// (a broadcast not requested by this client itself).
class MavlinkParameterClient {
public:
    enum class Result {
        Success,
        Timeout,
        ConnectionError,
        WrongType,
        ParamNameTooLong,
        NotFound,
        Failed,
        UnknownError
    };

    using SetParamCallback = std::function<void(Result)>;
    using GetParamAnyCallback = std::function<void(Result, ParamValue)>;
    using GetAllParamsCallback = std::function<void(Result, std::map<std::string, ParamValue>)>;
    using ParamChangedCallback = std::function<void(const std::string&, const ParamValue&)>;
    using TimeoutSCallback = std::function<double()>;

    // The actual message-sending function - wraps whatever write_message()/
    // send() you already have. Return true on success. No class/interface
    // needed at all, just a lambda, e.g.:
    //   [this](const mavlink_message_t& msg) { return write_message(msg) > 0; }
    using SendMessageCallback = std::function<bool(const mavlink_message_t&)>;

    static constexpr size_t PARAM_ID_LEN = 16;

    MavlinkParameterClient(
        asio::io_context& io_context,
        TimeoutHandler& timeout_handler,
        TimeoutSCallback timeout_s_callback,
        SendMessageCallback send_message_callback,
        uint8_t own_system_id,
        uint8_t own_component_id,
        uint8_t channel,
        uint8_t target_system_id,
        uint8_t target_component_id);
    ~MavlinkParameterClient();

    // Call this from YOUR OWN MAVLink message-handling loop, for EVERY
    // message received from the gimbal (not just PARAM_VALUE) - it filters
    // for the relevant msgid internally (currently: PARAM_VALUE) and ignores
    // everything else. Safe to call with any message, including non-PARAM_VALUE ones.
    //
    // Integration example inside Gimbal_Interface::messages_handler():
    //   case MAVLINK_MSG_ID_PARAM_VALUE:
    //       _param_client.handle_message(message);
    //       break;
    void handle_message(const mavlink_message_t& message);

    MavlinkParameterClient(const MavlinkParameterClient&) = delete;
    MavlinkParameterClient& operator=(const MavlinkParameterClient&) = delete;

    // --- Set ---
    Result set_param(const std::string& name, const ParamValue& value);
    void set_param_async(
        const std::string& name, const ParamValue& value, const SetParamCallback& callback,
        const void* cookie);

    Result set_param_int(const std::string& name, int32_t value);
    void set_param_int_async(
        const std::string& name, int32_t value, const SetParamCallback& callback,
        const void* cookie);

    Result set_param_float(const std::string& name, float value);
    void set_param_float_async(
        const std::string& name, float value, const SetParamCallback& callback,
        const void* cookie);

    // --- Get ---
    std::pair<Result, ParamValue> get_param(const std::string& name);
    void get_param_async(const std::string& name, const GetParamAnyCallback& callback, const void* cookie);

    std::pair<Result, int32_t> get_param_int(const std::string& name);
    void get_param_int_async(
        const std::string& name, const std::function<void(Result, int32_t)>& callback,
        const void* cookie);

    std::pair<Result, float> get_param_float(const std::string& name);
    void get_param_float_async(
        const std::string& name, const std::function<void(Result, float)>& callback,
        const void* cookie);

    // --- Get all ---
    std::pair<Result, std::map<std::string, ParamValue>> get_all_params();
    void get_all_params_async(GetAllParamsCallback callback, const void* cookie);

    // --- Read directly from the internal cache, WITHOUT sending any network request ---
    // The cache is populated after one successful call to
    // get_all_params()/get_all_params_async(), and is kept up to date every
    // time set_param_async(...) succeeds or a new PARAM_VALUE broadcast
    // arrives (subscribe_param_changed). Use these for high-frequency
    // reads/existence checks (every frame, every UI redraw...) without any
    // network round-trip.
    bool has_cached_param(const std::string& name) const;
    std::optional<ParamValue> get_cached_param(const std::string& name) const;
    std::optional<int32_t> get_cached_param_int(const std::string& name) const;
    std::optional<float> get_cached_param_float(const std::string& name) const;

    // --- Subscribe to param changes (broadcasts not tied to any request) ---
    void subscribe_param_changed(
        const std::string& name, const ParamChangedCallback& callback, const void* cookie);
    void unsubscribe_param_changed(const std::string& name, const void* cookie);
    void unsubscribe_all_params_changed(const void* cookie);

    void cancel_all_param(const void* cookie);
    void clear_cache();

private:
    // --- Work queue: only 1 item is processed at a time, to keep things
    // sequential with the PARAM protocol (autopilots often don't handle
    // overlapping requests well).
    struct WorkItemSet {
        std::string param_name;
        ParamValue param_value;
        SetParamCallback callback;
    };
    struct WorkItemGet {
        std::variant<std::string, int16_t> param_identifier;
        GetParamAnyCallback callback;
    };
    struct WorkItemGetAll {
        GetAllParamsCallback callback;
        uint16_t count{0};
        bool rerequesting{false};
    };
    using WorkItemVariant = std::variant<WorkItemSet, WorkItemGet, WorkItemGetAll>;

    struct WorkItem {
        WorkItem(WorkItemVariant variant, const void* c) :
            work_item_variant(std::move(variant)),
            cookie(c)
        {}
        WorkItemVariant work_item_variant;
        const void* cookie;
        bool already_requested{false};
        int retries_to_do{3};
    };

    struct Subscription {
        std::string param_name; // empty means "subscribe to all"
        ParamChangedCallback callback;
        const void* cookie;
    };

    void do_work();
    bool send_set_param_message(WorkItemSet& item);
    bool send_get_param_message(WorkItemGet& item);
    bool send_get_param_message(const std::array<char, PARAM_ID_LEN>& id_buf, int16_t index);
    bool send_request_list_message();
    bool request_next_missing(uint16_t count);

    void process_param_value(const mavlink_message_t& message);
    void receive_timeout();

    void find_and_call_subscriptions_value_changed(const std::string& id, const ParamValue& value);

    static bool validate_id_or_index(
        const std::variant<std::string, int16_t>& original, const std::string& id, int16_t index);
    static std::array<char, PARAM_ID_LEN> param_id_to_buffer(const std::string& name);
    static std::string extract_safe_param_id(const char* raw);
    // Accepts any cached/received int type (uint8/int16/int32/...) and
    // normalizes it to int32_t. Returns false if the ParamValue isn't an
    // integral type.
    static bool extract_int32(const ParamValue& value, int32_t& out);

    TimeoutHandler& _timeout_handler;
    TimeoutSCallback _timeout_s_callback;
    SendMessageCallback _send_message_callback;
    uint8_t _own_system_id;
    uint8_t _own_component_id;
    uint8_t _channel;
    uint8_t _target_system_id;
    uint8_t _target_component_id;
    asio::io_context& _io_context;

    std::deque<std::shared_ptr<WorkItem>> _work_queue;
    MavlinkParameterCache _param_cache;
    TimeoutCookie _timeout_cookie{};
    std::vector<Subscription> _subscriptions;

    static constexpr int _get_all_timeout_factor = 3;
};

} // namespace paramio