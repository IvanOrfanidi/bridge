#include <commands/camera_task_set_ev.hpp>
#include <commands/payload.hpp>

void CameraTaskSetEV::execute() {
    Logging::informationMessage("Command: 'camera set EV'", __FUNCTION__);

    CameraEvSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Camera EV settings: " << setting, __FUNCTION__);

        dji_osdk_ros::CameraEV cameraEv;
        cameraEv.request.payload_index = setting.payloadIndex; // Usually this is the camera index (for H20 = 0).
        cameraEv.request.exposure_mode = setting.exposureMode;
        cameraEv.request.exposure_compensation = setting.exposureCompensation;

        _client.call(cameraEv);

        _data.result = cameraEv.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}