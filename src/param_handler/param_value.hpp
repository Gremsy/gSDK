#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <variant>

// Requires the real MAVLink C headers (common/mavlink.h).
// https://github.com/mavlink/c_library_v2
#include <common/mavlink.h>

namespace paramio {

// Represents a single MAVLink parameter value, which can be any of the
// primitive types supported by the PARAM protocol (MAV_PARAM_TYPE).
class ParamValue {
public:
    using Variant = std::variant<
        uint8_t,
        int8_t,
        uint16_t,
        int16_t,
        uint32_t,
        int32_t,
        uint64_t,
        int64_t,
        float,
        double>;

    ParamValue() = default;

    template<class T>
    void set(T value)
    {
        _value = value;
    }

    template<class T>
    bool is() const
    {
        return std::holds_alternative<T>(_value);
    }

    template<class T>
    T get() const
    {
        return std::get<T>(_value);
    }

    bool is_same_type(const ParamValue& rhs) const { return _value.index() == rhs._value.index(); }

    // Overwrites the value with an int (regardless of which specific int
    // type is currently held), as long as the currently held type is
    // integral. Returns false if the currently held type is not integral
    // (e.g. float/double).
    bool set_int(int64_t value)
    {
        return std::visit(
            [&](auto&& held) -> bool {
                using T = std::decay_t<decltype(held)>;
                if constexpr (std::is_integral_v<T>) {
                    _value = static_cast<T>(value);
                    return true;
                } else {
                    return false;
                }
            },
            _value);
    }

    // Updates the value of *this with the value of other, keeping the type
    // of *this unchanged (used when the cache already knows the type and
    // the server returns a new value).
    void update_value_typesafe(const ParamValue& other) { _value = other._value; }

    bool operator==(const ParamValue& rhs) const { return _value == rhs._value; }
    bool operator!=(const ParamValue& rhs) const { return !(*this == rhs); }

    std::string get_string() const
    {
        std::ostringstream ss;
        std::visit([&](auto&& held) { ss << held; }, _value);
        return ss.str();
    }

    // --- MAVLink PARAM_VALUE / PARAM_SET (standard protocol, not extended) ---

    MAV_PARAM_TYPE get_mav_param_type() const
    {
        return std::visit(
            [](auto&& held) -> MAV_PARAM_TYPE {
                using T = std::decay_t<decltype(held)>;
                if constexpr (std::is_same_v<T, uint8_t>) return MAV_PARAM_TYPE_UINT8;
                else if constexpr (std::is_same_v<T, int8_t>) return MAV_PARAM_TYPE_INT8;
                else if constexpr (std::is_same_v<T, uint16_t>) return MAV_PARAM_TYPE_UINT16;
                else if constexpr (std::is_same_v<T, int16_t>) return MAV_PARAM_TYPE_INT16;
                else if constexpr (std::is_same_v<T, uint32_t>) return MAV_PARAM_TYPE_UINT32;
                else if constexpr (std::is_same_v<T, int32_t>) return MAV_PARAM_TYPE_INT32;
                else if constexpr (std::is_same_v<T, uint64_t>) return MAV_PARAM_TYPE_UINT64;
                else if constexpr (std::is_same_v<T, int64_t>) return MAV_PARAM_TYPE_INT64;
                else if constexpr (std::is_same_v<T, float>) return MAV_PARAM_TYPE_REAL32;
                else return MAV_PARAM_TYPE_REAL64;
            },
            _value);
    }

    // Encodes the value into the 4-byte float field of PARAM_SET/PARAM_VALUE
    // via a direct arithmetic assignment (float)value - the way the MAVLink
    // standard / most gimbal firmware does it (as opposed to the "bytewise"
    // encoding some autopilots use).
    float get_4_float_bytes() const
    {
        return std::visit([](auto&& held) -> float { return static_cast<float>(held); }, _value);
    }

    bool set_from_mavlink_param_value(const mavlink_param_value_t& msg)
    {
        return set_from_raw(static_cast<MAV_PARAM_TYPE>(msg.param_type), msg.param_value);
    }

    bool set_from_raw(MAV_PARAM_TYPE type, float raw)
    {
        switch (type) {
            case MAV_PARAM_TYPE_UINT8:
                _value = static_cast<uint8_t>(raw);
                return true;
            case MAV_PARAM_TYPE_INT8:
                _value = static_cast<int8_t>(raw);
                return true;
            case MAV_PARAM_TYPE_UINT16:
                _value = static_cast<uint16_t>(raw);
                return true;
            case MAV_PARAM_TYPE_INT16:
                _value = static_cast<int16_t>(raw);
                return true;
            case MAV_PARAM_TYPE_UINT32:
                _value = static_cast<uint32_t>(raw);
                return true;
            case MAV_PARAM_TYPE_INT32:
                _value = static_cast<int32_t>(raw);
                return true;
            case MAV_PARAM_TYPE_REAL32:
                _value = raw;
                return true;
            default:
                // UINT64/INT64/REAL64 cannot be represented in the 4-byte
                // float field of the standard protocol -> needs the
                // extended protocol.
                return false;
        }
    }

private:
    Variant _value{float{0.0f}};
};

} // namespace paramio