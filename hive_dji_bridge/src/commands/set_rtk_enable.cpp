#include <commands/set_rtk_enable.hpp>
#include <commands/payload.hpp>

void SetRtkEnable::execute() {
    Logging::informationMessage("Command: 'set rtk enable'", __FUNCTION__);

    dji_osdk_ros::SetRtkEnable rtkEnable;
    const auto valid = parsePayload(_payload, rtkEnable.request.enable);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "RTK settings: " << std::boolalpha << rtkEnable.request.enable,
                                    __FUNCTION__);

        _client.call(rtkEnable);

        _data.result = rtkEnable.response.result;
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}