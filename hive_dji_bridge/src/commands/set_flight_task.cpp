#include <commands/set_flight_task.hpp>
#include <commands/payload.hpp>

void SetFlightTask::execute() {
    Logging::informationMessage("Command: 'set flight task'", __FUNCTION__);

    FlightTaskControlSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Flight Task settings: " << setting, __FUNCTION__);

        dji_osdk_ros::FlightTaskControl flightTaskControl;
        flightTaskControl.request.task = setting.task;
        flightTaskControl.request.joystickCommand.x = setting.joystickCommandX;
        flightTaskControl.request.joystickCommand.y = setting.joystickCommandY;
        flightTaskControl.request.joystickCommand.z = setting.joystickCommandZ;
        flightTaskControl.request.joystickCommand.yaw = setting.joystickCommandYaw;
        flightTaskControl.request.velocityControlTimeMs = setting.velocityControlTimeMs;
        flightTaskControl.request.posThresholdInM = setting.posThresholdInM;
        flightTaskControl.request.yawThresholdInDeg = setting.yawThresholdInDeg;

        _client.call(flightTaskControl);

        _data.result = flightTaskControl.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}