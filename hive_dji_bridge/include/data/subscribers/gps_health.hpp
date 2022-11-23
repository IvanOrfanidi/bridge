#pragma once

#include <map>
#include <string>
#include <data/subscriber.hpp>

class GpsHealth : public Subscriber {
public:
    enum class SignalLevel : uint8_t {
        VERY_BAD = 0,
        VERY_WEAK = 1,
        WEAK = 2,        // At this level, the aircraft's go home functionality will still work
        GOOD = 3,        // At this level, the aircraft can hover in the air
        VERY_GOOD = 4,   // At this level, the aircraft can record the home point
        VERY_STRONG = 5, // At this level, the aircraft go mission
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(GpsHealth);
    }

    uint8_t getData() const {
        return std::any_cast<uint8_t>(getRawData());
    }

    SignalLevel getSignalLevel() const {
        return static_cast<SignalLevel>(getData());
    }

    std::string getSignalLevelInString() const {
        static const std::map<SignalLevel, std::string> mapOfMsgSignalLevel = {
          {SignalLevel::VERY_BAD, "very bad"},
          {SignalLevel::VERY_WEAK, "very weak"},
          {SignalLevel::WEAK, "weak"},
          {SignalLevel::GOOD, "good"},
          {SignalLevel::VERY_GOOD, "very good"},
          {SignalLevel::VERY_STRONG, "very strong"},
        };
        const auto it = mapOfMsgSignalLevel.find(getSignalLevel());
        return (it != mapOfMsgSignalLevel.end()) ? it->second : "UNKNOWN";
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const GpsHealth& gpsHealth) {
    const unsigned data = gpsHealth.getData();
    stream << std::make_tuple(data);
    return stream;
}