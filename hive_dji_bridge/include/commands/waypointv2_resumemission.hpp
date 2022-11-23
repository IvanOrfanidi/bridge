#pragma once

#include <dji_osdk_ros/ResumeWaypointV2Mission.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class ResumeWaypointV2Mission
  : public Command
  , public RosServiceClient<dji_osdk_ros::ResumeWaypointV2Mission> {
public:
    explicit ResumeWaypointV2Mission(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/dji_osdk_ros/waypointV2_resumeMission"}) {
    }

    void execute() override;
};