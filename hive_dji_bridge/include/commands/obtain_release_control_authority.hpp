#pragma once

#include <dji_osdk_ros/ObtainControlAuthority.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class ObtainReleaseControlAuthority
  : public Command
  , public RosServiceClient<dji_osdk_ros::ObtainControlAuthority> {
public:
    explicit ObtainReleaseControlAuthority(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/obtain_release_control_authority"}) {
    }

    void execute() override;
};