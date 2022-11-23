#include <commands/waypointv2_resumemission.hpp>
#include <commands/payload.hpp>

void ResumeWaypointV2Mission::execute() {
    Logging::informationMessage("Command: 'waypointv2 resume mission'", __FUNCTION__);

    dji_osdk_ros::ResumeWaypointV2Mission resumeWaypointV2Mission;

    _client.call(resumeWaypointV2Mission);

    _data.result = resumeWaypointV2Mission.response.result;

    Logging::informationMessage(drone::utils::make_string() << "Result: " << std::boolalpha << _data.result, __FUNCTION__);
}