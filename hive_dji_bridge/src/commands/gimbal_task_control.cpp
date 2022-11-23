#include <commands/gimbal_task_control.hpp>
#include <commands/payload.hpp>

void GimbalTaskControl::execute() {
    Logging::informationMessage("Command: 'gimbal task control'", __FUNCTION__);

    GimbalTaskControlSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Gimbal Control settings: " << setting, __FUNCTION__);

        dji_osdk_ros::GimbalAction gimbalAction;
        gimbalAction.request.is_reset = setting.isReset;
        gimbalAction.request.payload_index = setting.payloadIndex;
        gimbalAction.request.rotationMode = setting.rotationMode;
        gimbalAction.request.pitch = setting.pitch;
        gimbalAction.request.roll = setting.roll;
        gimbalAction.request.yaw = setting.yaw;
        gimbalAction.request.time = setting.time;
        gimbalAction.request.reference = setting.reference;
        gimbalAction.request.atti_or_joint_ref = setting.attiOrJointRef;

        _client.call(gimbalAction);

        _data.result = gimbalAction.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}