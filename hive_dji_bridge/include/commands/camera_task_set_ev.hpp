#pragma once

#include <dji_osdk_ros/CameraEV.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class CameraTaskSetEV
  : public Command
  , public RosServiceClient<dji_osdk_ros::CameraEV> {
public:
    explicit CameraTaskSetEV(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/camera_task_set_EV"}) {
    }

    void execute() override;
};