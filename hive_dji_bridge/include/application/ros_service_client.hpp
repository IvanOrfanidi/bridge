#pragma once

#include <ros/ros.h>
#include <application/ros_items.hpp>
#include <application/logging.hpp>
#include <common/make_string.hpp>

template<class ServiceClient>
class RosServiceClient {
protected:
    // timeout - the amount of time to wait for before timing out. If timeout is -1 (default), waits until the node is shutdown
    explicit RosServiceClient(const RosServiceName& serviceName, const ros::Duration timeout = ros::Duration(-1))
      : _client(_node.serviceClient<ServiceClient>(serviceName.data())) {
        waitForExistence(timeout, serviceName.data());
    }

    // timeout - the amount of time to wait for before timing out. If timeout is -1 (default), waits until the node is shutdown
    explicit RosServiceClient(const RosNode& node,
                              const RosServiceName& serviceName,
                              const ros::Duration timeout = ros::Duration(-1))
      : _node(node.data())
      , _client(_node.serviceClient<ServiceClient>(serviceName.data())) {
        waitForExistence(timeout, serviceName.data());
    }

    // Wait for this service to be advertised and available. Blocks until it is.
    void waitForExistence(const ros::Duration timeout, const std::string_view serviceName) {
        if (!_client.waitForExistence(timeout)) {
            Logging::errorMessage(drone::utils::make_string()
                                    << "Failed to create \'" << serviceName.data() << "\' service within timeout " << timeout,
                                  __FUNCTION__);
        }
    }

    ros::NodeHandle _node{"~"};
    ros::ServiceClient _client;
};
