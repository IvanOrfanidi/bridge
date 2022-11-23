#pragma once

#include <limits>

#include <ros/ros.h>
#include <common/utils.hpp>
#include <data/subscriber.hpp>

struct CameraTaskGetApertureData {
    unsigned payloadIndex = 0;
    unsigned aperture = std::numeric_limits<unsigned>::max();
};

class CameraTaskGetApertureService : public Subscriber {
public:
    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(CameraTaskGetApertureService);
    }

    CameraTaskGetApertureData getData() const {
        return std::any_cast<CameraTaskGetApertureData>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraTaskGetApertureService& service) {
    const auto data = service.getData();
    stream << drone::utils::to_tuple(data);
    return stream;
}