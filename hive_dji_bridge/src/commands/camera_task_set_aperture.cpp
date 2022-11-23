#include <commands/camera_task_set_aperture.hpp>
#include <commands/payload.hpp>

void CameraTaskSetAperture::execute() {
    Logging::informationMessage("Command: 'camera task set aperture'", __FUNCTION__);

    CameraApertureSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Camera Aperture settings: " << setting, __FUNCTION__);

        dji_osdk_ros::CameraAperture cameraAperture;
        cameraAperture.request.payload_index = setting.payloadIndex; // Usually this is the camera index (for H20 = 0).
        cameraAperture.request.exposure_mode = setting.exposureMode;
        cameraAperture.request.aperture = setting.aperture;

        _client.call(cameraAperture);

        _data.result = cameraAperture.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}