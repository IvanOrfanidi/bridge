#pragma once

#include <nlohmann/json.hpp>
#include <application/logging.hpp>
#include <common/utils.hpp>
#include <common/operators.hpp>
#include <common/make_string.hpp>

struct SimulationSetting {
    unsigned enable = 0;
    double longitude = 0.0;
    double latitude = 0.0;
};
template<class StreamType>
StreamType& operator<<(StreamType& stream, const SimulationSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct GimbalTaskControlSetting {
    unsigned isReset = 0;
    unsigned payloadIndex = 0;
    unsigned rotationMode = 0;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    double time = 0.0;
    unsigned reference = 0;
    unsigned attiOrJointRef = 0;
};
template<class StreamType>
StreamType& operator<<(StreamType& stream, const GimbalTaskControlSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct CameraOpticalZoomFactorSetting {
    unsigned payloadIndex = 0;
    double factor = 0.0;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraOpticalZoomFactorSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct CameraZoomCtrltSetting {
    unsigned startStop = 0;
    unsigned payloadIndex = 0;
    unsigned direction = 0;
    unsigned speed = 0;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraZoomCtrltSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct FlightTaskControlSetting {
    unsigned task = 0;
    float joystickCommandX = 0.0f;
    float joystickCommandY = 0.0f;
    float joystickCommandZ = 0.0f;
    float joystickCommandYaw = 0.0f;
    uint32_t velocityControlTimeMs = 0;
    float posThresholdInM = 0.0f;
    float yawThresholdInDeg = 0.0f;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const FlightTaskControlSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct CameraRecordVideoSetting {
    unsigned payloadIndex = 0;
    unsigned startStop = 0;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraRecordVideoSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct CameraEvSetting {
    unsigned payloadIndex = 0;
    unsigned exposureMode = 0;
    unsigned exposureCompensation = 0;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraEvSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct CameraIsoSetting {
    unsigned payloadIndex = 0;
    unsigned exposureMode = 0;
    unsigned isoData = 0;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraIsoSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct CameraApertureSetting {
    unsigned payloadIndex = 0;
    unsigned exposureMode = 0;
    uint16_t aperture = 0;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraApertureSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct CameraFocusPointSetting {
    unsigned payloadIndex = 0;
    float x = 0.0f;
    float y = 0.0f;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraFocusPointSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct CameraTapZoomPointSetting {
    unsigned payloadIndex = 0;
    unsigned multiplier = 0;
    float x = 0.0f;
    float y = 0.0f;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CameraTapZoomPointSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

struct ChangeCameraH264SourceSetting {
    unsigned requestView = 0;
    unsigned source = 0;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const ChangeCameraH264SourceSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

/********* RC Map (A3) *********
*
*       -10000  <--->  0      <---> 10000
* MODE: API(F)  <---> ATTI(A) <--->  POS (P)
*
*        CH3 +10000                     CH1 +10000
*               ^                              ^
*               |                              |                   / -5000
*    CH2        |                   CH0        |                  /
*  -10000 <-----------> +10000    -10000 <-----------> +10000    H
*               |                              |                  \
*               |                              |                   \ -10000
*               V                              V
*            -10000                         -10000
*
*   In this code, before publishing, we normalize RC
*****************************/
struct FlightControlSetpointGenericSetting {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;
    float mode = 0.0f;
    float gear = 0.0f;
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const FlightControlSetpointGenericSetting& setting) {
    stream << drone::utils::to_tuple(setting);
    return stream;
}

/* Parsers */
template<class SettingType>
bool parsePayload(const std::string_view data, SettingType& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting = json["value"].get<SettingType>();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}
bool parsePayload(const std::string_view, SimulationSetting&);
bool parsePayload(const std::string_view, GimbalTaskControlSetting&);
bool parsePayload(const std::string_view, CameraOpticalZoomFactorSetting&);
bool parsePayload(const std::string_view, CameraZoomCtrltSetting&);
bool parsePayload(const std::string_view, FlightTaskControlSetting&);
bool parsePayload(const std::string_view, CameraEvSetting&);
bool parsePayload(const std::string_view, CameraRecordVideoSetting&);
bool parsePayload(const std::string_view, CameraIsoSetting&);
bool parsePayload(const std::string_view, CameraApertureSetting&);
bool parsePayload(const std::string_view, CameraFocusPointSetting&);
bool parsePayload(const std::string_view, CameraTapZoomPointSetting&);
bool parsePayload(const std::string_view, ChangeCameraH264SourceSetting&);
bool parsePayload(const std::string_view, FlightControlSetpointGenericSetting&);