#include <commands/waypointv2_startmission.hpp>
#include <commands/payload.hpp>

void StartWaypointV2Mission::execute() {
    Logging::informationMessage("Command: 'waypointv2 start mission'", __FUNCTION__);

    dji_osdk_ros::StartWaypointV2Mission startWaypointV2Mission;

    _client.call(startWaypointV2Mission);

    _data.result = startWaypointV2Mission.response.result;

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}