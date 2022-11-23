#pragma once

#include <data/subscriber.hpp>

class HeightAboveTakeoff : public Subscriber {
public:
    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(HeightAboveTakeoff);
    }

    float getData() const {
        return std::any_cast<float>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const HeightAboveTakeoff& heightAboveTakeoff) {
    stream << std::make_tuple(heightAboveTakeoff.getData());
    return stream;
}