#pragma once

#include <ros/ros.h>
#include <common/utils.hpp>
#include <data/subscriber.hpp>
#include <common/math.hpp>

struct BatteryInfoData {
    uint16_t remainFlyTime = 0;
    uint16_t goHomeNeedTime = 0;
    uint16_t landNeedTime = 0;
    uint16_t goHomeNeedCapacity = 0;
    uint16_t landNeedCapacity = 0;
    float safeFlyRadius = 0.0f;
    float capacityConsumeSpeed = 0.0f;
    unsigned goHomeCountDownState = 0;
    unsigned gohomeCountDownvalue = 0;
    uint16_t voltage = 0;
    unsigned batteryCapacityPercentage = 0;
    unsigned lowBatteryAlarmThreshold = 0;
    unsigned lowBatteryAlarmEnable = 0;
    unsigned seriousLowBatteryAlarmThreshold = 0;
    unsigned seriousLowBatteryAlarmEnable = 0;
};

inline bool operator==(const BatteryInfoData& left, const BatteryInfoData& right) {
    // clang-format off
    return (left.remainFlyTime == right.remainFlyTime &&
            left.goHomeNeedTime == right.goHomeNeedTime &&
            left.landNeedTime == right.landNeedTime &&
            left.goHomeNeedCapacity == right.goHomeNeedCapacity &&
            left.landNeedCapacity == right.landNeedCapacity &&
            drone::math::isEqual(left.safeFlyRadius, right.safeFlyRadius) &&
            drone::math::isEqual(left.capacityConsumeSpeed, right.capacityConsumeSpeed) &&
            left.goHomeCountDownState == right.goHomeCountDownState &&
            left.gohomeCountDownvalue == right.gohomeCountDownvalue &&
            left.voltage == right.voltage &&
            left.batteryCapacityPercentage == right.batteryCapacityPercentage &&
            left.lowBatteryAlarmThreshold == right.lowBatteryAlarmThreshold &&
            left.lowBatteryAlarmEnable == right.lowBatteryAlarmEnable &&
            left.seriousLowBatteryAlarmThreshold == right.seriousLowBatteryAlarmThreshold &&
            left.seriousLowBatteryAlarmEnable == right.seriousLowBatteryAlarmEnable);
    // clang-format on
}

inline bool operator!=(const BatteryInfoData& left, const BatteryInfoData& right) {
    return !(left == right);
}

struct BatteryStateData {
    unsigned voltageNotSafety = 0;
    unsigned veryLowVoltageAlarm = 0;
    unsigned lowVoltageAlarm = 0;
    unsigned seriousLowCapacityAlarm = 0;
    unsigned lowCapacityAlarm = 0;
};

inline bool operator==(const BatteryStateData& left, const BatteryStateData& right) {
    // clang-format off
    return (left.voltageNotSafety == right.voltageNotSafety &&
            left.veryLowVoltageAlarm == right.veryLowVoltageAlarm &&
            left.lowVoltageAlarm == right.lowVoltageAlarm &&
            left.seriousLowCapacityAlarm == right.seriousLowCapacityAlarm &&
            left.lowCapacityAlarm == right.lowCapacityAlarm);
    // clang-format on
}

inline bool operator!=(const BatteryStateData& left, const BatteryStateData& right) {
    return !(left == right);
}

struct WholeBatteryInfoData {
    BatteryInfoData info{};
    BatteryStateData state{};
};

inline bool operator==(const WholeBatteryInfoData& left, const WholeBatteryInfoData& right) {
    return (left.info == right.info && left.state == right.state);
}
inline bool operator!=(const WholeBatteryInfoData& left, const WholeBatteryInfoData& right) {
    return !(left == right);
}

class GetWholeBatteryInfoService : public Subscriber {
public:
    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(GetWholeBatteryInfoService);
    }

    WholeBatteryInfoData getData() const {
        return std::any_cast<WholeBatteryInfoData>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const GetWholeBatteryInfoService& service) {
    const auto data = service.getData();
    stream << drone::utils::to_tuple(data.info) << drone::utils::to_tuple(data.state);
    return stream;
}