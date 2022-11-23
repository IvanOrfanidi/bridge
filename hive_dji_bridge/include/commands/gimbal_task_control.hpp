#pragma once

#include <dji_osdk_ros/GimbalAction.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class GimbalTaskControl
  : public Command
  , public RosServiceClient<dji_osdk_ros::GimbalAction> {
public:
    explicit GimbalTaskControl(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/gimbal_task_control"}) {
    }

    void execute() override;
};