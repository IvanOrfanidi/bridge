#pragma once

#include <dji_osdk_ros/SetSimulationOn.h>

#include <application/ros_service_client.hpp>
#include <commands/command.hpp>

class SetSimulationOn
  : public Command
  , public RosServiceClient<dji_osdk_ros::SetSimulationOn> {
public:
    explicit SetSimulationOn(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosServiceClient(RosServiceName{"/dji_osdk_ros/set_simulation_on"}) {
    }

    void execute() override;

private:
    bool setSimulationOff();
};