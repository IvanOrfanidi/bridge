#include <commands/camera_task_zoom_ctrl.hpp>
#include <commands/payload.hpp>

void CameraZoomCtrl::execute() {
    Logging::informationMessage("Command: 'camera task tap zoom point'", __FUNCTION__);

    CameraZoomCtrltSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Camera Zoom Ctrlt settings: " << setting, __FUNCTION__);

        dji_osdk_ros::CameraZoomCtrl cameraZoomCtrl;
        cameraZoomCtrl.request.start_stop = setting.startStop;
        cameraZoomCtrl.request.payload_index = setting.payloadIndex; // Usually this is the camera index (for H20 = 0).
        cameraZoomCtrl.request.direction = setting.direction;
        cameraZoomCtrl.request.speed = setting.speed;

        _client.call(cameraZoomCtrl);

        _data.result = cameraZoomCtrl.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}