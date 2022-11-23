#pragma once

#include <map>
#include <string>
#include <dji_status.hpp>
#include <data/subscriber.hpp>

class DisplayMode : public Subscriber {
public:
    enum class Mode : uint8_t {
        MANUAL_CONTROL = DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL,
        ATTITUDE = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE,
        P_GPS = DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS,
        HOTPOINT = DJI::OSDK::VehicleStatus::DisplayMode::MODE_HOTPOINT_MODE,
        ASSISTED_TAKEOFF = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF,
        AUTO_TAKEOFF = DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF,
        AUTO_LANDING = DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING,
        ON_MISSION = DJI::OSDK::VehicleStatus::DisplayMode::MODE_RESERVED_14,
        GO_HOME = DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME,
        SDK_CTRL = DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL,
        SPORT = DJI::OSDK::VehicleStatus::DisplayMode::MODE_RESERVED_21,
        SPORT_2 = DJI::OSDK::VehicleStatus::DisplayMode::MODE_RESERVED_31,
        FORCE_AUTO_LANDING = DJI::OSDK::VehicleStatus::DisplayMode::MODE_FORCE_AUTO_LANDING,
        TRIPOD = DJI::OSDK::VehicleStatus::DisplayMode::MODE_RESERVED_38,
        SEARCH = DJI::OSDK::VehicleStatus::DisplayMode::MODE_SEARCH_MODE,
        ENGINE_START = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START,
        SMART_TRACK = 51,
        THREE_PROPELLER_EMERGENCY_LANDING = 52,
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(DisplayMode);
    }

    uint8_t getData() const {
        return std::any_cast<uint8_t>(getRawData());
    }

    Mode getDisplayMode() const {
        return static_cast<Mode>(getData());
    }

    std::string displayModeToString() const {
        static const std::map<Mode, std::string> mapOfMsgDisplayModes{
          {Mode::MANUAL_CONTROL, "manual control"},
          {Mode::ATTITUDE, "attitude"},
          {Mode::P_GPS, "pgps"},
          {Mode::HOTPOINT, "hotpoint"},
          {Mode::ASSISTED_TAKEOFF, "assisted takeoff"},
          {Mode::AUTO_TAKEOFF, "auto takeoff"},
          {Mode::AUTO_LANDING, "auto landing"},
          {Mode::ON_MISSION, "on mission"},
          {Mode::GO_HOME, "navi go home"},
          {Mode::SDK_CTRL, "sdk ctrl"},
          {Mode::SPORT, "sport"},
          {Mode::SPORT_2, "sport2"},
          {Mode::FORCE_AUTO_LANDING, "force auto landing"},
          {Mode::TRIPOD, "tripod control"},
          {Mode::SEARCH, "search"},
          {Mode::ENGINE_START, "engine start"},
          {Mode::SMART_TRACK, "smart track"},
          {Mode::THREE_PROPELLER_EMERGENCY_LANDING, "three-propeller emergency landing"},
        };
        const auto displayMode = getDisplayMode();
        const auto it = mapOfMsgDisplayModes.find(displayMode);
        return (it != mapOfMsgDisplayModes.end()) ? it->second : "UNKNOWN";
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const DisplayMode& displayMode) {
    const unsigned data = displayMode.getData();
    stream << std::make_tuple(data);
    return stream;
}