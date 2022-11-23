#pragma once

#include <data/quaternion.hpp>
#include <data/subscriber.hpp>
#include <data/header.hpp>

class Attitude : public Subscriber {
public:
    struct Data {
        Header header{};
        Quaternion<double> quaternion{0.0, 0.0, 0.0, 0.0};
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(Attitude);
    }

    Data getData() const {
        return std::any_cast<Data>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const Attitude& attitude) {
    const Attitude::Data data = attitude.getData();
    stream << "Header: " << data.header << ", Quaternion: " << data.quaternion;
    return stream;
}