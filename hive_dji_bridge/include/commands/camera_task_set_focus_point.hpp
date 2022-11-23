#pragma once

#include <dji_osdk_ros/CameraFocusPoint.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class CameraFocusPoint
  : public Command
  , public RosServiceClient<dji_osdk_ros::CameraFocusPoint> {
public:
    explicit CameraFocusPoint(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/camera_task_set_focus_point"}) {
    }

    void execute() override;
};