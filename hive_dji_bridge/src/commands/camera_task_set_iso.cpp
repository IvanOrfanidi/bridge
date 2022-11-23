#include <commands/camera_task_set_iso.hpp>
#include <commands/payload.hpp>

void CameraTaskSetISO::execute() {
    Logging::informationMessage("Command: 'camera task set EV'", __FUNCTION__);

    CameraIsoSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Camera ISO settings: " << setting, __FUNCTION__);

        dji_osdk_ros::CameraISO cameraIso;
        cameraIso.request.payload_index = setting.payloadIndex; // Usually this is the camera index (for H20 = 0).
        cameraIso.request.exposure_mode = setting.exposureMode;
        cameraIso.request.iso_data = setting.isoData;

        _client.call(cameraIso);

        _data.result = cameraIso.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}