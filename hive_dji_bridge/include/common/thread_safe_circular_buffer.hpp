#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <boost/circular_buffer.hpp>

namespace drone {
namespace multithread {

template<typename T>
class circular_buffer {
public:
    circular_buffer& operator=(const circular_buffer&) = delete;

    explicit circular_buffer(uint16_t size)
      : _data(size) {
    }

    explicit circular_buffer(const circular_buffer& other) {
        std::lock_guard<std::mutex> lock(_mutex);
        _data = other._data;
    }

    void push_back(const T value) {
        std::lock_guard<std::mutex> lock(_mutex);
        _data.push_back(value);
        _cv.notify_one();
    }

    void push_front(const T value) {
        std::lock_guard<std::mutex> lock(_mutex);
        _data.push_front(value);
        _cv.notify_one();
    }

    void wait_and_pop_front(T& value) {
        std::unique_lock<std::mutex> lock(_mutex);
        _cv.wait(lock, [this] { return !_data.empty(); });
        value = _data.front();
        _data.pop_front();
    }

    void wait_and_pop_back(T& value) {
        std::unique_lock<std::mutex> lock(_mutex);
        _cv.wait(lock, [this] { return !_data.empty(); });
        value = _data.back();
        _data.pop_back();
    }

    bool try_pop_front(T& value) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_data.empty()) {
            return false;
        }
        value = _data.front();
        _data.pop_front();
        return true;
    }

    bool try_pop_back(T& value) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_data.empty()) {
            return false;
        }
        value = _data.back();
        _data.pop_back();
        return true;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _data.empty();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _data.size();
    }

private:
    boost::circular_buffer<T> _data;
    mutable std::mutex _mutex;
    std::condition_variable _cv;
};

} // namespace multithread
} // namespace drone