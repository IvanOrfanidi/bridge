#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>

namespace drone {
namespace multithread {

template<typename T>
class queue {
public:
    queue() = default;
    queue& operator=(const queue&) = delete;

    explicit queue(const queue& other) {
        std::lock_guard<std::mutex> lock(_mutex);
        _data = other._data;
    }

    void push(const T value) {
        std::lock_guard<std::mutex> lock(_mutex);
        _data.push(value);
        _cv.notify_one();
    }

    void wait_and_pop(T& value) {
        std::unique_lock<std::mutex> lock(_mutex);
        _cv.wait(lock, [this] { return !_data.empty(); });
        value = _data.front();
        _data.pop();
    }

    std::shared_ptr<T> wait_and_pop() {
        std::unique_lock<std::mutex> lock(_mutex);
        _cv.wait(lock, [this] { return !_data.empty(); });
        auto res = std::make_shared<T>(_data.front());
        _data.pop();
        return res;
    }

    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_data.empty()) {
            return false;
        }
        value = _data.front();
        _data.pop();
        return true;
    }

    std::shared_ptr<T> try_pop() {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_data.empty()) {
            return std::shared_ptr<T>();
        }
        auto res = std::make_shared<T>(_data.front());
        _data.pop();
        return res;
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
    std::queue<T> _data;
    mutable std::mutex _mutex;
    std::condition_variable _cv;
};

} // namespace multithread
} // namespace drone