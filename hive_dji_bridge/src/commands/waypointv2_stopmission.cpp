#include <commands/waypointv2_stopmission.hpp>
#include <commands/payload.hpp>

void StopWaypointV2Mission::execute() {
    Logging::informationMessage("Command: 'waypointv2 stop mission'", __FUNCTION__);

    dji_osdk_ros::StopWaypointV2Mission stopWaypointV2Mission;

    _client.call(stopWaypointV2Mission);

    _data.result = stopWaypointV2Mission.response.result;

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}