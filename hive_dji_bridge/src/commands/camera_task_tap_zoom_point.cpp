#include <commands/camera_task_tap_zoom_point.hpp>
#include <commands/payload.hpp>

void CameraTapZoomPoint::execute() {
    Logging::informationMessage("Command: 'camera task tap zoom point'", __FUNCTION__);

    CameraTapZoomPointSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Camera Tap Zoom Point settings: " << setting, __FUNCTION__);

        dji_osdk_ros::CameraTapZoomPoint cameraTapZoomPoint;
        cameraTapZoomPoint.request.payload_index = setting.payloadIndex; // Usually this is the camera index (for H20 = 0).
        cameraTapZoomPoint.request.multiplier = setting.multiplier;
        cameraTapZoomPoint.request.x = setting.x;
        cameraTapZoomPoint.request.y = setting.y;

        _client.call(cameraTapZoomPoint);

        _data.result = cameraTapZoomPoint.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}