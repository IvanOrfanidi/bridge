#pragma once

#include <map>
#include <data/subscriber.hpp>

class FlightAnomaly : public Subscriber {
public:
    enum Anomaly : uint32_t {
        IMPACT_IN_AIR = (1 << 0),
        RANDOM_FLY = (1 << 1),
        VERTICAL_CONTROL_FAIL = (1 << 2),
        HORIZONTAL_CONTROL_FAIL = (1 << 3),
        YAW_CONTROL_FAIL = (1 << 4),
        AIRCRAFT_IS_FALLING = (1 << 5),
        STRONG_WIND_LEVEL1 = (1 << 6),
        STRONG_WIND_LEVEL2 = (1 << 7),
        COMPASS_INSTALLATION_ERROR = (1 << 8),
        IMU_INSTALLATION_ERROR = (1 << 9),
        ESC_TEMPERATURE_HIGH = (1 << 10),
        ESC_DISCONNECTED = (1 << 11),
        GPS_YAW_ERROR = (1 << 12),
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(FlightAnomaly);
    }

    uint32_t getData() const {
        return std::any_cast<uint32_t>(getRawData());
    }

    static std::string flightAnomalyToString(uint32_t flightAnomaly) {
        static const std::map<Anomaly, std::string> mapOfMsgFlightAnomaly = {
          {IMPACT_IN_AIR, "impact in air"},
          {RANDOM_FLY, "random fly"},
          {VERTICAL_CONTROL_FAIL, "vertical control fail"},
          {HORIZONTAL_CONTROL_FAIL, "horizontal control fail"},
          {YAW_CONTROL_FAIL, "yaw control fail"},
          {AIRCRAFT_IS_FALLING, "aircraft is falling"},
          {STRONG_WIND_LEVEL1, "strong wind level1"},
          {STRONG_WIND_LEVEL2, "strong wind level2"},
          {COMPASS_INSTALLATION_ERROR, "compass installation error"},
          {IMU_INSTALLATION_ERROR, "imu installation error"},
          {ESC_TEMPERATURE_HIGH, "esc temperature high"},
          {ESC_DISCONNECTED, "esc disconnected"},
          {GPS_YAW_ERROR, "gps yaw error"},
        };
        std::string out;
        for (const auto& [key, anomaly] : mapOfMsgFlightAnomaly) {
            if (flightAnomaly & key) {
                out += anomaly + " & ";
            }
        }
        if (out.empty()) {
            return "UNKNOWN";
        }
        out.erase(out.size() - 3);
        return out;
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const FlightAnomaly& flightAnomaly) {
    stream << std::make_tuple(flightAnomaly.getData());
    return stream;
}