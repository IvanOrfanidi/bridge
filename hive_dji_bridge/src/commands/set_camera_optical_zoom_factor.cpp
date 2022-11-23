#include <commands/set_camera_optical_zoom_factor.hpp>
#include <commands/payload.hpp>

void SetCameraOpticalZoomFactor::execute() {
    Logging::informationMessage("Command: 'gimbal task control'", __FUNCTION__);

    CameraOpticalZoomFactorSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Camera Zoom settings: " << setting, __FUNCTION__);

        dji_osdk_ros::SetCameraOpticalZoomFactor cameraOpticalZoomFactor;
        cameraOpticalZoomFactor.request.payloadIndex = setting.payloadIndex; // Usually this is the camera index (for H20 = 0).
        cameraOpticalZoomFactor.request.factor = setting.factor;

        _client.call(cameraOpticalZoomFactor);

        _data.result = cameraOpticalZoomFactor.response.success;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}