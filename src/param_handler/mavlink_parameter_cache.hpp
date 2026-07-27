#pragma once

#include <algorithm>
#include <cassert>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "param_value.hpp"

namespace paramio {

// Caches all parameters received from the autopilot during a GetAllParams
// session, and tracks which indices are still missing so they can be
// re-requested (retransmission).
class MavlinkParameterCache {
public:
    struct Param {
        std::string id;
        ParamValue value;
        uint16_t index;
    };

    enum class AddNewParamResult { Ok, AlreadyExists, TooManyParams };
    enum class UpdateExistingParamResult { Ok, MissingParam, WrongType };

    AddNewParamResult add_new_param(const std::string& param_id, ParamValue value, int16_t index);
    UpdateExistingParamResult update_existing_param(const std::string& param_id, ParamValue value);

    std::vector<Param> all_parameters() const { return _all_params; }
    std::map<std::string, ParamValue> all_parameters_map() const;

    std::optional<Param> param_by_id(const std::string& param_id) const;
    std::optional<Param> param_by_index(uint16_t param_index) const;

    uint16_t count() const;
    void clear();

    bool exists(const std::string& param_id) const;
    bool exists(uint16_t param_index) const;

    // Number of indices in [0, count) that are not yet in the cache.
    uint16_t missing_count(uint16_t count) const;

    // Returns up to `chunk_size` missing indices in [0, count), used to send
    // PARAM_REQUEST_READ requests in batches. Updates _last_missing_requested.
    std::vector<uint16_t> next_missing_indices(uint16_t count, uint16_t chunk_size);

    std::optional<uint16_t> last_missing_requested() const { return _last_missing_requested; }

    void print_missing(uint16_t count) const;

private:
    std::vector<Param> _all_params;
    std::optional<uint16_t> _last_missing_requested;
};

} // namespace paramio