#pragma once

#include <dji_osdk_ros/CameraAperture.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class CameraTaskSetAperture
  : public Command
  , public RosServiceClient<dji_osdk_ros::CameraAperture> {
public:
    explicit CameraTaskSetAperture(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/camera_task_set_aperture"}) {
    }

    void execute() override;
};