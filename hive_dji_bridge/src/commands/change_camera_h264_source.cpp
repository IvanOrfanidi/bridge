#include <commands/change_camera_h264_source.hpp>
#include <commands/payload.hpp>

void ChangeCameraH264Source::execute() {
    Logging::informationMessage("Command: 'change camera h264 source'", __FUNCTION__);

    ChangeCameraH264SourceSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Change Camera H264 Source settings: " << setting,
                                    __FUNCTION__);

        dji_osdk_ros::ChangeCameraH264Source changeCameraH264Source;
        changeCameraH264Source.request.request_view = setting.requestView;
        changeCameraH264Source.request.source = setting.source;

        _client.call(changeCameraH264Source);

        _data.result = changeCameraH264Source.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}