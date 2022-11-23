#pragma once

#include <sensor_msgs/Joy.h>

#include <application/ros_publisher.hpp>
#include <commands/command.hpp>

class FlightControlSetpointGeneric
  : public Command
  , public RosPublisher<sensor_msgs::Joy> {
public:
    explicit FlightControlSetpointGeneric(const CommandId& id, const CommandPayload& payload)
      : Command(id, payload)
      , RosPublisher(RosPublisherName{"/dji_osdk_ros/flight_control_setpoint_generic"}) {
    }

    void execute() override;
};