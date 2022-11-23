#pragma once

#include <any>
#include <typeinfo>
#include <typeindex>
#include <common/thread_safe_circular_buffer.hpp>

class Subscriber {
public:
    static constexpr uint16_t DEFAULT_DATA_BUFFER_SIZE = 8;

    virtual std::type_index getType() = 0;

    void notify(const std::any data) {
        _dataBuffer.push_back(data);
    }

    std::any getRawData() const {
        std::any data;
        BOOST_ASSERT_MSG(_dataBuffer.try_pop_front(data), "The subscriber has no data");
        return data;
    }

    bool haveData() const {
        return !_dataBuffer.empty();
    }

protected:
    Subscriber() = default;
    explicit Subscriber(uint16_t bufferSize)
      : _dataBuffer(bufferSize) {
    }

    virtual ~Subscriber() = default;

private:
    mutable drone::multithread::circular_buffer<std::any> _dataBuffer{DEFAULT_DATA_BUFFER_SIZE};
};