#pragma once

#include <limits>

#include <ros/ros.h>
#include <common/utils.hpp>
#include <data/subscriber.hpp>

struct CameraTaskGetIsoData {
    unsigned payloadIndex = 0;
    unsigned isoData = std::numeric_limits<unsigned>::max();
};

class CameraTaskGetIsoService : public Subscriber {
public:
    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(CameraTaskGetIsoService);
    }

    CameraTaskGetIsoData getData() const {
        return std::any_cast<CameraTaskGetIsoData>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraTaskGetIsoService& service) {
    const auto data = service.getData();
    stream << drone::utils::to_tuple(data);
    return stream;
}