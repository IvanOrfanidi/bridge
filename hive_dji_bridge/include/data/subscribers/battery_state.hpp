#pragma once

#include <map>
#include <data/subscriber.hpp>
#include <data/header.hpp>
#include <common/utils.hpp>

class BatteryState : public Subscriber {
public:
    enum class PowerSupplyStatus : uint8_t {
        UNKNOWN = 0,
        CHARGING = 1,
        DISCHARGING = 2,
        NOT_CHARGING = 3,
        FULL = 4,
    };

    enum class PowerSupplyHealth : uint8_t {
        UNKNOWN = 0,
        GOOD = 1,
        OVERHEAT = 2,
        DEAD = 3,
        OVERVOLTAGE = 4,
        UNSPEC_FAILURE = 5,
        COLD = 6,
        WATCHDOG_TIMER_EXPIRE = 7,
        SAFETY_TIMER_EXPIRE = 8,
    };

    enum class PowerSupplyTechnology : uint8_t {
        UNKNOWN = 0,
        NIMH = 1,
        LION = 2,
        LIPO = 3,
        LIFE = 4,
        NICD = 5,
        LIMN = 6,
    };

    struct Data {
        Header header{};
        float voltage = 0.0F;
        float current = 0.0F;
        float charge = 0.0F;
        float capacity = 0.0F;
        float designCapacity = 0.0F;
        float percentage = 0.0F;
        PowerSupplyStatus powerSupplyStatus = PowerSupplyStatus::UNKNOWN;
        PowerSupplyHealth powerSupplyHealth = PowerSupplyHealth::UNKNOWN;
        PowerSupplyTechnology powerSupplyTechnology = PowerSupplyTechnology::UNKNOWN;
        uint8_t present = 0;
        std::string location{};
        std::string serialNumber{};
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(BatteryState);
    }

    Data getData() const {
        return std::any_cast<Data>(getRawData());
    }

    static std::string getPowerSupplyStatusToString(PowerSupplyStatus powerSupplyStatus) {
        static const std::map<PowerSupplyStatus, std::string> mapOfMsgPowerSupplyStatus = {
          {PowerSupplyStatus::UNKNOWN, "unknown"},
          {PowerSupplyStatus::CHARGING, "charging"},
          {PowerSupplyStatus::DISCHARGING, "discharging"},
          {PowerSupplyStatus::NOT_CHARGING, "not charging"},
          {PowerSupplyStatus::FULL, "full"},
        };
        const auto it = mapOfMsgPowerSupplyStatus.find(powerSupplyStatus);
        return (it != mapOfMsgPowerSupplyStatus.end()) ? it->second : "UNKNOWN";
    }

    static std::string powerSupplyHealthToString(PowerSupplyHealth powerSupplyHealth) {
        static const std::map<PowerSupplyHealth, std::string> mapOfMsgPowerSupplyHealth = {
          {PowerSupplyHealth::UNKNOWN, "unknown"},
          {PowerSupplyHealth::GOOD, "good"},
          {PowerSupplyHealth::OVERHEAT, "overheat"},
          {PowerSupplyHealth::DEAD, "dead"},
          {PowerSupplyHealth::OVERVOLTAGE, "overvoltage"},
          {PowerSupplyHealth::UNSPEC_FAILURE, "unspec failure"},
          {PowerSupplyHealth::COLD, "cold"},
          {PowerSupplyHealth::WATCHDOG_TIMER_EXPIRE, "watchdog timer expire"},
          {PowerSupplyHealth::SAFETY_TIMER_EXPIRE, "safety timer expire"},
        };
        const auto it = mapOfMsgPowerSupplyHealth.find(powerSupplyHealth);
        return (it != mapOfMsgPowerSupplyHealth.end()) ? it->second : "UNKNOWN";
    }

    static std::string powerSupplyTechnologyToString(PowerSupplyTechnology powerSupplyTechnology) {
        static const std::map<PowerSupplyTechnology, std::string> mapOfMsgPowerSupplyTechnology = {
          {PowerSupplyTechnology::UNKNOWN, "unknown"},
          {PowerSupplyTechnology::NIMH, "nimh"},
          {PowerSupplyTechnology::LION, "lion"},
          {PowerSupplyTechnology::LIPO, "lipo"},
          {PowerSupplyTechnology::LIFE, "life"},
          {PowerSupplyTechnology::NICD, "nicd"},
          {PowerSupplyTechnology::LIMN, "limn"},
        };
        const auto it = mapOfMsgPowerSupplyTechnology.find(powerSupplyTechnology);
        return (it != mapOfMsgPowerSupplyTechnology.end()) ? it->second : "UNKNOWN";
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const BatteryState& batteryState) {
    const auto data = batteryState.getData();
    const auto state = std::make_tuple(data.voltage,
                                       data.current,
                                       data.charge,
                                       data.capacity,
                                       data.designCapacity,
                                       data.percentage,
                                       static_cast<unsigned>(data.powerSupplyStatus),
                                       static_cast<unsigned>(data.powerSupplyHealth),
                                       static_cast<unsigned>(data.powerSupplyTechnology),
                                       static_cast<unsigned>(data.present),
                                       data.location,
                                       data.serialNumber);

    stream << "Header: " << data.header << ", State: " << state;
    return stream;
}