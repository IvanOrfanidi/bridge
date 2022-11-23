#pragma once

#include <dji_osdk_ros/StopWaypointV2Mission.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class StopWaypointV2Mission
  : public Command
  , public RosServiceClient<dji_osdk_ros::StopWaypointV2Mission> {
public:
    explicit StopWaypointV2Mission(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/dji_osdk_ros/waypointV2_stopMission"}) {
    }

    void execute() override;
};