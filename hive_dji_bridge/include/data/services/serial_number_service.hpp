#pragma once

#include <ros/ros.h>
#include <data/subscriber.hpp>

class SerialNumberService : public Subscriber {
public:
    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(SerialNumberService);
    }

    std::string getData() const {
        return std::any_cast<std::string>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const SerialNumberService& service) {
    stream << std::make_tuple(service.getData());
    return stream;
}