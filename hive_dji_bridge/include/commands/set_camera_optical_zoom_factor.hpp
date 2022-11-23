#pragma once

#include <dji_osdk_ros/SetCameraOpticalZoomFactor.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class SetCameraOpticalZoomFactor
  : public Command
  , public RosServiceClient<dji_osdk_ros::SetCameraOpticalZoomFactor> {
public:
    explicit SetCameraOpticalZoomFactor(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/dji_osdk_ros/set_camera_optical_zoom_factor"}) {
    }

    void execute() override;
};