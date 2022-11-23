#include <commands/waypointv2_pausemission.hpp>
#include <commands/payload.hpp>

void PauseWaypointV2Mission::execute() {
    Logging::informationMessage("Command: 'waypointv2 pause mission'", __FUNCTION__);

    dji_osdk_ros::PauseWaypointV2Mission pauseWaypointV2Mission;

    _client.call(pauseWaypointV2Mission);

    _data.result = pauseWaypointV2Mission.response.result;

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}