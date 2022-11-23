#include <commands/set_simulation_on.hpp>
#include <commands/payload.hpp>

void SetSimulationOn::execute() {
    Logging::informationMessage("Command: 'set simulation on'", __FUNCTION__);

    SimulationSetting setting;
    const auto valid = parsePayload(_payload, setting);
    if (valid) {
        Logging::informationMessage(drone::utils::make_string() << "Simulation settings: " << setting, __FUNCTION__);
        _data.result = setSimulationOff();
        if (setting.enable != 0) {
            dji_osdk_ros::SetSimulationOn setSimulationOn;
            setSimulationOn.request.enable = setting.enable;
            setSimulationOn.request.longtitude = setting.longitude;
            setSimulationOn.request.lattitude = setting.latitude;

            _client.call(setSimulationOn);

            _data.result = setSimulationOn.response.result;
        }
    } else {
        errorIncorrectPayload();
    }

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}

bool SetSimulationOn::setSimulationOff() {
    dji_osdk_ros::SetSimulationOn setSimulationOn;
    setSimulationOn.request.enable = 0;
    setSimulationOn.request.longtitude = 0;
    setSimulationOn.request.lattitude = 0;

    _client.call(setSimulationOn);

    return setSimulationOn.response.result;
}