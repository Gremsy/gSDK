#include "mavlink_parameter_cache.hpp"
#include <iostream>

namespace paramio {

MavlinkParameterCache::AddNewParamResult MavlinkParameterCache::add_new_param(
    const std::string& param_id, ParamValue value, int16_t index)
{
    if (exists(param_id)) {
        return AddNewParamResult::AlreadyExists;
    }

    if (static_cast<size_t>(_all_params.size() + 1) >
        static_cast<size_t>(std::numeric_limits<int16_t>::max())) {
        return AddNewParamResult::TooManyParams;
    }

    _all_params.push_back(Param{
        param_id,
        std::move(value),
        (index != -1 ? static_cast<uint16_t>(index) : static_cast<uint16_t>(_all_params.size()))});
    return AddNewParamResult::Ok;
}

MavlinkParameterCache::UpdateExistingParamResult MavlinkParameterCache::update_existing_param(
    const std::string& param_id, ParamValue value)
{
    auto it = std::find_if(_all_params.begin(), _all_params.end(), [&](const auto& param) {
        return param_id == param.id;
    });

    if (it == _all_params.end()) {
        return UpdateExistingParamResult::MissingParam;
    }

    if (!it->value.is_same_type(value)) {
        return UpdateExistingParamResult::WrongType;
    }

    it->value.update_value_typesafe(value);
    return UpdateExistingParamResult::Ok;
}

std::map<std::string, ParamValue> MavlinkParameterCache::all_parameters_map() const
{
    std::map<std::string, ParamValue> mp;
    for (const auto& param : _all_params) {
        mp.insert({param.id, param.value});
    }
    return mp;
}

std::optional<MavlinkParameterCache::Param>
MavlinkParameterCache::param_by_id(const std::string& param_id) const
{
    for (const auto& param : _all_params) {
        if (param.id == param_id) {
            return param;
        }
    }
    return {};
}

std::optional<MavlinkParameterCache::Param>
MavlinkParameterCache::param_by_index(uint16_t param_index) const
{
    for (const auto& param : _all_params) {
        if (param.index == param_index) {
            return param;
        }
    }
    return {};
}

uint16_t MavlinkParameterCache::count() const
{
    assert(_all_params.size() < std::numeric_limits<uint16_t>::max());
    return static_cast<uint16_t>(_all_params.size());
}

void MavlinkParameterCache::clear()
{
    _all_params.clear();
    _last_missing_requested.reset();
}

bool MavlinkParameterCache::exists(const std::string& param_id) const
{
    return std::any_of(_all_params.begin(), _all_params.end(), [&](const auto& param) {
        return param_id == param.id;
    });
}

bool MavlinkParameterCache::exists(uint16_t param_index) const
{
    return std::any_of(_all_params.begin(), _all_params.end(), [&](const auto& param) {
        return param_index == param.index;
    });
}

uint16_t MavlinkParameterCache::missing_count(uint16_t count) const
{
    uint16_t missing = 0;
    for (uint16_t i = 0; i < count; ++i) {
        if (!exists(i)) ++missing;
    }
    return missing;
}

std::vector<uint16_t>
MavlinkParameterCache::next_missing_indices(uint16_t count, uint16_t chunk_size)
{
    std::vector<uint16_t> result;
    for (uint16_t i = 0; i < count; ++i) {
        if (exists(i)) continue;

        result.push_back(i);
        _last_missing_requested = i;

        if (result.size() == chunk_size) break;
    }
    return result;
}

void MavlinkParameterCache::print_missing(uint16_t count) const
{
    std::cerr << "Available: " << _all_params.size() << " / " << count << "\n";
    for (uint16_t i = 0; i < count; ++i) {
        if (!exists(i)) {
            std::cerr << "  missing index: " << i << "\n";
        }
    }
}

} // namespace paramio
