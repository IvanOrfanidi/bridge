#pragma once

#include <ros/ros.h>
#include <string>

class Config {
public:
    static Config& instance() {
        static Config instance;
        return instance;
    }

    std::string gatewayUrl() const noexcept {
        return _gatewayUrl;
    }

private:
    Config() {
        if (!ros::ok()) {
            throw std::runtime_error("ROS is not running");
        }

        ros::NodeHandle node{};
        const bool result = node.getParam("gateway_url", _gatewayUrl);
        if (!result || _gatewayUrl.empty()) {
            throw std::runtime_error("Failed to get gateway URL");
        }
        _gatewayUrl.append("/dji");
    }

    std::string _gatewayUrl{};
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const Config& config) {
    stream << "gateway url: {" << config.gatewayUrl() << '}';
    return stream;
}
