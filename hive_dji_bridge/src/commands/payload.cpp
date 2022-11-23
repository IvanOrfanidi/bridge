#include <limits>
#include <commands/payload.hpp>

bool parsePayload(const std::string_view data, SimulationSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.enable = json["enable"].get<uint8_t>();
        setting.longitude = json.contains("longitude") ? json["longitude"].get<double>() : 0.0;
        setting.latitude = json.contains("latitude") ? json["latitude"].get<double>() : 0.0;
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, GimbalTaskControlSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.isReset = json["is_reset"].get<uint8_t>();
        setting.payloadIndex = json.contains("payload_index") ? json["payload_index"].get<uint8_t>() : 0;
        setting.rotationMode = json.contains("rotation_mode") ? json["rotation_mode"].get<uint8_t>() : 0;
        setting.pitch = json.contains("pitch") ? json["pitch"].get<float>() : 0.0f;
        setting.roll = json.contains("roll") ? json["roll"].get<float>() : 0.0f;
        setting.yaw = json.contains("yaw") ? json["yaw"].get<float>() : 0.0f;
        setting.time = json.contains("time") ? json["time"].get<double>() : 0.0;
        setting.reference = json.contains("reference") ? json["reference"].get<uint8_t>() : 0;
        setting.attiOrJointRef = json.contains("atti_or_joint_ref") ? json["atti_or_joint_ref"].get<uint8_t>() : 0;
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, CameraOpticalZoomFactorSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.payloadIndex = json["payload_index"].get<uint8_t>();
        setting.factor = json["factor"].get<double>();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, FlightTaskControlSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.task = json["task"].get<uint8_t>();
        setting.joystickCommandX = json.contains("joystick_command_x") ? json["joystick_command_x"].get<float>() : 0.0f;
        setting.joystickCommandY = json.contains("joystick_command_y") ? json["joystick_command_y"].get<float>() : 0.0f;
        setting.joystickCommandZ = json.contains("joystick_command_z") ? json["joystick_command_z"].get<float>() : 0.0f;
        setting.joystickCommandYaw = json.contains("joystick_command_yaw") ? json["joystick_command_yaw"].get<float>() : 0.0f;
        setting.velocityControlTimeMs =
          json.contains("velocity_control_time_ms") ? json["velocity_control_time_ms"].get<uint32_t>() : 0;
        setting.posThresholdInM = json.contains("pos_threshold_in_m") ? json["pos_threshold_in_m"].get<float>() : 0.0f;
        setting.yawThresholdInDeg = json.contains("yaw_threshold_in_deg") ? json["yaw_threshold_in_deg"].get<float>() : 0.0f;
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, CameraRecordVideoSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.payloadIndex = json["payload_index"].get<uint8_t>();
        setting.startStop = json["start_stop"].get<uint8_t>();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, CameraEvSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.payloadIndex = json["payload_index"].get<uint8_t>();
        setting.exposureMode = json["exposure_mode"].get<uint8_t>();
        setting.exposureCompensation = json["exposure_compensation"].get<uint8_t>();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, CameraIsoSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.payloadIndex = json["payload_index"].get<uint8_t>();
        setting.exposureMode = json["exposure_mode"].get<uint8_t>();
        setting.isoData = json["iso_data"].get<uint8_t>();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, CameraApertureSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.payloadIndex = json["payload_index"].get<uint8_t>();
        setting.exposureMode = json["exposure_mode"].get<uint8_t>();
        setting.aperture = json["aperture"].get<uint16_t>();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, CameraFocusPointSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.payloadIndex = json["payload_index"].get<uint8_t>();
        setting.x = json["x"].get<float>();
        setting.y = json["y"].get<float>();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, CameraTapZoomPointSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.payloadIndex = json["payload_index"].get<uint8_t>();
        setting.multiplier = json["multiplier"].get<uint8_t>();
        setting.x = json["x"].get<float>();
        setting.y = json["y"].get<float>();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, CameraZoomCtrltSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.startStop = json["start_stop"].get<uint8_t>();
        setting.payloadIndex = json["payload_index"].get<uint8_t>();
        setting.direction = json["direction"].get<uint8_t>();
        setting.speed = json["speed"].get<uint8_t>();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, ChangeCameraH264SourceSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.requestView = json["request_view"].get<uint8_t>();
        setting.source = json["source"].get<uint8_t>();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}

bool parsePayload(const std::string_view data, FlightControlSetpointGenericSetting& setting) {
    try {
        const auto json = nlohmann::json::parse(data.data());
        setting.roll = json.contains("roll") ? json["roll"].get<float>() : 0.0f;
        setting.pitch = json.contains("pitch") ? json["pitch"].get<float>() : 0.0f;
        setting.yaw = json.contains("yaw") ? json["yaw"].get<float>() : 0.0f;
        setting.throttle = json.contains("throttle") ? json["throttle"].get<float>() : 0.0f;
        setting.mode = json.contains("mode") ? json["mode"].get<float>() : 0.0f;
        setting.gear = json.contains("gear") ? json["gear"].get<float>() : std::numeric_limits<float>::min();
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Incorrect payload: " << data, __FUNCTION__);
        return false;
    }
    return true;
}