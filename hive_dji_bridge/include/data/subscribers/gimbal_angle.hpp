#pragma once

#include <data/subscriber.hpp>
#include <data/header.hpp>
#include <data/vector.hpp>

class GimbalAngle : public Subscriber {
public:
    struct Data {
        Header header{};
        Vector3<double> vector{0.0, 0.0, 0.0};
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(GimbalAngle);
    }

    Data getData() const {
        return std::any_cast<Data>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const GimbalAngle& gimbalAngle) {
    const auto data = gimbalAngle.getData();
    stream << "Header: " << data.header << ", Vector: " << data.vector;
    return stream;
}