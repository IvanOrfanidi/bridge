#include <commands/camera_record_video_action.hpp>
#include <commands/payload.hpp>

void CameraRecordVideoAction::execute() {
    Logging::informationMessage("Command: 'camera record video action'", __FUNCTION__);

    CameraRecordVideoSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Camera Shoot Photo settings: " << setting, __FUNCTION__);

        dji_osdk_ros::CameraRecordVideoAction cameraRecordVideoAction;
        cameraRecordVideoAction.request.payload_index = setting.payloadIndex; // Usually this is the camera index (for H20 = 0).
        cameraRecordVideoAction.request.start_stop = setting.startStop;

        _client.call(cameraRecordVideoAction);

        _data.result = cameraRecordVideoAction.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}