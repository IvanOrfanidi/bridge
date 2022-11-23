#pragma once

#include <limits>

#include <ros/ros.h>
#include <common/utils.hpp>
#include <data/subscriber.hpp>

struct CameraTaskGetEvData {
    unsigned payloadIndex = 0;
    unsigned exposureCompensation = std::numeric_limits<unsigned>::max();
};

class CameraTaskGetEvService : public Subscriber {
public:
    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(CameraTaskGetEvService);
    }

    CameraTaskGetEvData getData() const {
        return std::any_cast<CameraTaskGetEvData>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraTaskGetEvService& service) {
    const auto data = service.getData();
    stream << drone::utils::to_tuple(data);
    return stream;
}