#pragma once

#include <dji_osdk_ros/CameraStartShootSinglePhoto.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class CameraStartShootSinglePhoto
  : public Command
  , public RosServiceClient<dji_osdk_ros::CameraStartShootSinglePhoto> {
public:
    explicit CameraStartShootSinglePhoto(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/camera_start_shoot_single_photo"}) {
    }

    void execute() override;
};