#pragma once

#include <dji_osdk_ros/PauseWaypointV2Mission.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class PauseWaypointV2Mission
  : public Command
  , public RosServiceClient<dji_osdk_ros::PauseWaypointV2Mission> {
public:
    explicit PauseWaypointV2Mission(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/dji_osdk_ros/waypointV2_pauseMission"}) {
    }

    void execute() override;
};