#pragma once

#include <dji_osdk_ros/CameraISO.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class CameraTaskSetISO
  : public Command
  , public RosServiceClient<dji_osdk_ros::CameraISO> {
public:
    explicit CameraTaskSetISO(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/camera_task_set_ISO"}) {
    }

    void execute() override;
};