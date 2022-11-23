#pragma once

#include <dji_osdk_ros/FlightTaskControl.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class SetFlightTask
  : public Command
  , public RosServiceClient<dji_osdk_ros::FlightTaskControl> {
public:
    explicit SetFlightTask(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/flight_task_control"}) {
    }

    void execute() override;
};