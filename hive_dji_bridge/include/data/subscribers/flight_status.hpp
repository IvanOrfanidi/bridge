#pragma once

#include <map>
#include <string>

#include <data/subscriber.hpp>
#include <dji_status.hpp>

class FlightStatus : public Subscriber {
public:
    enum class Status : uint8_t {
        STOPPED = DJI::OSDK::VehicleStatus::FlightStatus::STOPED,
        ON_GROUND = DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND,
        IN_AIR = DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(FlightStatus);
    }

    uint8_t getData() const {
        return std::any_cast<uint8_t>(getRawData());
    }

    Status getStatus() const {
        return static_cast<Status>(getData());
    }

    std::string getStatusInString() const {
        static const std::map<Status, std::string> mapOfMsgFStatus = {
          {Status::STOPPED, "stopped"},
          {Status::ON_GROUND, "on ground"},
          {Status::IN_AIR, "in air"},
        };
        const auto it = mapOfMsgFStatus.find(getStatus());
        return (it != mapOfMsgFStatus.end()) ? it->second : "UNKNOWN";
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const FlightStatus& flightStatus) {
    const unsigned data = flightStatus.getData();
    stream << std::make_tuple(data);
    return stream;
}
