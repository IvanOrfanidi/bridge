#pragma once

#include <ros/ros.h>
#include <application/ros_items.hpp>

template<class Publisher>
class RosPublisher {
protected:
    explicit RosPublisher(const RosPublisherName& publisherName)
      : _publisher(_node.advertise<Publisher>(publisherName.data(), _queueSize)) {
    }

    explicit RosPublisher(const RosNode& node, const RosPublisherName& publisherName)
      : _node(node.data())
      , _publisher(_node.advertise<Publisher>(publisherName.data(), _queueSize)) {
    }

    explicit RosPublisher(const RosPublisherName& publisherName, uint32_t queueSize)
      : _queueSize(queueSize)
      , _publisher(_node.advertise<Publisher>(publisherName.data(), _queueSize)) {
    }

    explicit RosPublisher(const RosNode& node, const RosPublisherName& publisherName, uint32_t queueSize)
      : _queueSize(queueSize)
      , _node(node.data())
      , _publisher(_node.advertise<Publisher>(publisherName.data(), _queueSize)) {
    }

    const uint32_t _queueSize = 10;
    ros::NodeHandle _node{"~"};
    const ros::Publisher _publisher;
};
