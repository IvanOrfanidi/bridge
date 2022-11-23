#pragma once

#include <dji_osdk_ros/CameraZoomCtrl.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class CameraZoomCtrl
  : public Command
  , public RosServiceClient<dji_osdk_ros::CameraZoomCtrl> {
public:
    explicit CameraZoomCtrl(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/camera_task_zoom_ctrl"}) {
    }

    void execute() override;
};