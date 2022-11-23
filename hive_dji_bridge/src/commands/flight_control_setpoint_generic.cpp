#include <common/math.hpp>
#include <commands/flight_control_setpoint_generic.hpp>
#include <commands/payload.hpp>

void FlightControlSetpointGeneric::execute() {
    Logging::informationMessage("Command: 'flight control setpoint generic'", __FUNCTION__);

    FlightControlSetpointGenericSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Flight Control Setpoint Generic settings: " << setting,
                                    __FUNCTION__);

        sensor_msgs::Joy rc;
        rc.header.stamp = ros::Time::now();
        rc.header.frame_id = "rc";
        rc.axes = {
          setting.roll,
          setting.pitch,
          setting.yaw,
          setting.throttle,
          setting.mode,
        };
        if (drone::math::isEqual(setting.gear,
                                 std::numeric_limits<float>::min())) { // In old version, parameter gear was not passed.
            rc.axes.push_back(setting.gear);
        }

        _publisher.publish(rc);

        _data.result = true;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}