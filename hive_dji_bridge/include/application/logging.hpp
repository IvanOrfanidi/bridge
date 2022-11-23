#pragma once

#include <ros/ros.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <common/make_string.hpp>
#include <common/thread_safe_circular_buffer.hpp>

#define INFO(node, fun, msg, ...) ROS_INFO(node, fun, msg, ##__VA_ARGS__)
#define ERROR(node, fun, msg, ...) ROS_ERROR(node, fun, msg, ##__VA_ARGS__)
#define WARN(node, fun, msg, ...) ROS_WARN(node, fun, msg, ##__VA_ARGS__)

class Logging {
public:
    static constexpr uint16_t DEFAULT_LOG_BUFFER_SIZE = 64;

    enum class Level {
        INFORMATION,
        WARNING,
        ERROR,
        DEBUG, // No used in message
    };

    struct Data {
        Level level = Level::DEBUG;
        boost::posix_time::ptime dataTime{};
        std::string function{};
        std::string message{};
    };

    static void registerNode(const std::string_view nodeName) {
        _nodeName = nodeName.data();
    }

    static std::string getNodeName() {
        return _nodeName;
    }

    static void informationMessage(const std::string& message, const std::string& function, bool isPushData = true) {
        INFO("[%s][%s] %s", _nodeName.data(), function.c_str(), message.c_str());

        if (isPushData) {
            pushData(message, function, Level::INFORMATION);
        }
    }

    static void warningMessage(const std::string& message, const std::string& function, bool isPushData = true) {
        WARN("[%s][%s] %s", _nodeName.data(), function.c_str(), message.c_str());

        if (isPushData) {
            pushData(message, function, Level::WARNING);
        }
    }

    static void errorMessage(const std::string& message, const std::string& function, bool isPushData = true) {
        ERROR("[%s][%s] %s", _nodeName.data(), function.c_str(), message.c_str());

        if (isPushData) {
            pushData(message, function, Level::ERROR);
        }
    }

    static std::shared_ptr<drone::multithread::circular_buffer<Data>> getLogPool() {
        return _logBuffer;
    }

private:
    static void pushData(const std::string& message, const std::string& function, const Level level) {
        const Data logData{
          level,
          boost::posix_time::second_clock::local_time(),
          function,
          message,
        };
        _logBuffer->push_back(std::move(logData));
    }

    static inline std::shared_ptr<drone::multithread::circular_buffer<Data>> _logBuffer =
      std::make_shared<drone::multithread::circular_buffer<Data>>(DEFAULT_LOG_BUFFER_SIZE);
    static inline std::string _nodeName{};
};