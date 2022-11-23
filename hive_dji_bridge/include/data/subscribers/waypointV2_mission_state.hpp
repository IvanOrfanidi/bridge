#pragma once

#include <data/subscriber.hpp>
#include <common/utils.hpp>

class WaypointV2MissionState : public Subscriber {
public:
    struct Data {
        int8_t commonDataVersion = 0;
        uint16_t commonDataLen = 0;
        uint16_t curWaypointIndex = 0;
        uint8_t state = 0;
        uint16_t velocity = 0;
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(WaypointV2MissionState);
    }

    Data getData() const {
        return std::any_cast<Data>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const WaypointV2MissionState& mission) {
    const auto data = mission.getData();
    stream << std::make_tuple(static_cast<int>(data.commonDataVersion),
                              data.commonDataLen,
                              data.curWaypointIndex,
                              static_cast<unsigned>(data.state),
                              data.velocity);
    return stream;
}