#pragma once

#include <ros/ros.h>
#include <common/utils.hpp>
#include <data/subscriber.hpp>

struct SingleBatteryDynamicInfo {
    unsigned batteryIndex = 0;
    int32_t currentVoltage = 0;
    int32_t currentElectric = 0;
    uint32_t fullCapacity = 0;
    uint32_t remainedCapacity = 0;
    int16_t batteryTemperature = 0;
    unsigned cellCount = 0;
    unsigned batteryCapacityPercent = 0;
    unsigned sop = 0;
};

inline bool operator==(const SingleBatteryDynamicInfo& left, const SingleBatteryDynamicInfo& right) {
    // clang-format off
    return (left.batteryIndex == right.batteryIndex &&
            left.currentVoltage == right.currentVoltage &&
            left.currentElectric == right.currentElectric &&
            left.fullCapacity == right.fullCapacity &&
            left.remainedCapacity == right.remainedCapacity &&
            left.batteryTemperature == right.batteryTemperature &&
            left.cellCount == right.cellCount &&
            left.batteryCapacityPercent == right.batteryCapacityPercent &&
            left.sop == right.sop);
    // clang-format on
}

inline bool operator!=(const SingleBatteryDynamicInfo& left, const SingleBatteryDynamicInfo& right) {
    return !(left == right);
}

struct SingleBatteryDynamicState {
    unsigned cellBreak = 0;
    unsigned selfCheckError = 0;
    unsigned batteryClosedReason = 0;
    unsigned batSOHState = 0;
    unsigned maxCycleLimit = 0;
    unsigned hasCellBreak = 0;
    unsigned heatState = 0;
};

inline bool operator==(const SingleBatteryDynamicState& left, const SingleBatteryDynamicState& right) {
    // clang-format off
    return (left.cellBreak == right.cellBreak &&
            left.selfCheckError == right.selfCheckError &&
            left.batteryClosedReason == right.batteryClosedReason &&
            left.batSOHState == right.batSOHState &&
            left.maxCycleLimit == right.maxCycleLimit &&
            left.hasCellBreak == right.hasCellBreak &&
            left.heatState == right.heatState);
    // clang-format on
}

inline bool operator!=(const SingleBatteryDynamicState& left, const SingleBatteryDynamicState& right) {
    return !(left == right);
}

struct SingleBatteryDynamicInfoData {
    SingleBatteryDynamicInfo info{};
    SingleBatteryDynamicState state{};
};

inline bool operator==(const SingleBatteryDynamicInfoData& left, const SingleBatteryDynamicInfoData& right) {
    return (left.info == right.info && left.state == right.state);
}
inline bool operator!=(const SingleBatteryDynamicInfoData& left, const SingleBatteryDynamicInfoData& right) {
    return !(left == right);
}

class GetSingleBatteryDynamicInfoService : public Subscriber {
public:
    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(GetSingleBatteryDynamicInfoService);
    }

    SingleBatteryDynamicInfoData getData() const {
        return std::any_cast<SingleBatteryDynamicInfoData>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const GetSingleBatteryDynamicInfoService& service) {
    const auto data = service.getData();
    stream << drone::utils::to_tuple(data.info) << drone::utils::to_tuple(data.state);
    return stream;
}