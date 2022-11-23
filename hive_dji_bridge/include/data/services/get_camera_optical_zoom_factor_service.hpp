#pragma once

#include <limits>

#include <ros/ros.h>
#include <common/utils.hpp>
#include <data/subscriber.hpp>

struct GetCameraOpticalZoomFactorData {
    unsigned payloadIndex = 0;
    double factor = std::numeric_limits<double>::max();
};

class GetCameraOpticalZoomFactorService : public Subscriber {
public:
    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(GetCameraOpticalZoomFactorService);
    }

    GetCameraOpticalZoomFactorData getData() const {
        return std::any_cast<GetCameraOpticalZoomFactorData>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const GetCameraOpticalZoomFactorService& service) {
    const auto data = service.getData();
    stream << drone::utils::to_tuple(data);
    return stream;
}