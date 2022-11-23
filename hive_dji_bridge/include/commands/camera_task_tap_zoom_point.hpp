#pragma once

#include <dji_osdk_ros/CameraTapZoomPoint.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class CameraTapZoomPoint
  : public Command
  , public RosServiceClient<dji_osdk_ros::CameraTapZoomPoint> {
public:
    explicit CameraTapZoomPoint(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/camera_task_tap_zoom_point"}) {
    }

    void execute() override;
};