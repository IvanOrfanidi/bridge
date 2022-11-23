#pragma once

#include <dji_osdk_ros/ChangeCameraH264Source.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class ChangeCameraH264Source
  : public Command
  , public RosServiceClient<dji_osdk_ros::ChangeCameraH264Source> {
public:
    explicit ChangeCameraH264Source(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/camera_record_video_action"}) {
    }

    void execute() override;
};