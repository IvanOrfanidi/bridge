#include <commands/camera_task_set_focus_point.hpp>
#include <commands/payload.hpp>

void CameraFocusPoint::execute() {
    Logging::informationMessage("Command: 'camera task set focus point'", __FUNCTION__);

    CameraFocusPointSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Camera Focus Point settings: " << setting, __FUNCTION__);

        dji_osdk_ros::CameraFocusPoint cameraFocusPoint;
        cameraFocusPoint.request.payload_index = setting.payloadIndex; // Usually this is the camera index (for H20 = 0).
        cameraFocusPoint.request.x = setting.x;
        cameraFocusPoint.request.y = setting.y;

        _client.call(cameraFocusPoint);

        _data.result = cameraFocusPoint.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}