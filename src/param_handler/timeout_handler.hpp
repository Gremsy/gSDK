#pragma once

#include <asio.hpp>
#include <functional>
#include <map>
#include <mutex>

namespace paramio {

// Identifies a pending timeout, used to remove/refresh it later.
using TimeoutCookie = uint64_t;

// Manages timeouts based on asio::steady_timer. Every callback runs on the
// io_context passed to the constructor -> safe to call from inside a handler
// of that same io_context.
class TimeoutHandler {
public:
    explicit TimeoutHandler(asio::io_context& io_context) : _io_context(io_context) {}

    // Registers a new timeout; the callback runs after `duration_s` seconds
    // unless it's removed/refreshed before then. Returns a cookie for
    // managing it later.
    TimeoutCookie add(std::function<void()> callback, double duration_s)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        const auto cookie = _next_cookie++;
        auto entry = std::make_shared<Entry>();
        entry->timer = std::make_shared<asio::steady_timer>(_io_context);
        entry->callback = std::move(callback);
        entry->duration_s = duration_s;

        arm(entry, cookie);
        _timers[cookie] = entry;
        return cookie;
    }

    void remove(TimeoutCookie cookie)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _timers.find(cookie);
        if (it != _timers.end()) {
            it->second->timer->cancel();
            _timers.erase(it);
        }
    }

    // Extends the current timeout (resets the clock with the same
    // callback/duration), without creating a new cookie.
    void refresh(TimeoutCookie cookie)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _timers.find(cookie);
        if (it == _timers.end()) {
            return;
        }
        arm(it->second, cookie);
    }

private:
    struct Entry {
        std::shared_ptr<asio::steady_timer> timer;
        std::function<void()> callback;
        double duration_s{};
    };

    void arm(const std::shared_ptr<Entry>& entry, TimeoutCookie cookie)
    {
        entry->timer->expires_after(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(entry->duration_s)));

        entry->timer->async_wait([this, cookie](const asio::error_code& ec) {
            if (ec) {
                // Timer was cancelled (remove/refresh) -> don't call the callback.
                return;
            }
            std::function<void()> cb;
            {
                std::lock_guard<std::mutex> lock2(_mutex);
                auto it = _timers.find(cookie);
                if (it == _timers.end()) return;
                cb = it->second->callback;
                _timers.erase(it);
            }
            if (cb) cb();
        });
    }

    asio::io_context& _io_context;
    std::mutex _mutex;
    std::map<TimeoutCookie, std::shared_ptr<Entry>> _timers;
    TimeoutCookie _next_cookie{1};
};

} // namespace paramio