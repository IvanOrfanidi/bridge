#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <future>
#include <unordered_map>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <dji_osdk_ros/WaypointV2MissionEventPush.h>
#include <dji_osdk_ros/WaypointV2MissionStatePush.h>
#include <dji_osdk_ros/FlightAnomaly.h>

#include <application/service_pool.hpp>
#include <data/subscriber.hpp>
#include <common/thread_safe_queue.hpp>
#include <commands/command.hpp>

class Handler {
public:
    static Handler& instance();

    void initialize();

    void start(std::shared_future<void>);

    void attachCommandQueue(const std::shared_ptr<drone::multithread::queue<std::shared_ptr<Command>>>);

    void attachSubscriber(const std::shared_ptr<Subscriber>& subscriber);

    void detachSubscriber(const std::shared_ptr<Subscriber>& subscriber);

    void testSubscriberCallback(int);

    void notifyResult(const std::type_index, const std::any) const;

private:
    static constexpr size_t QUEUE_SIZE = 1;

    Handler() = default;

    void gpsHealthCallback(const std_msgs::UInt8::ConstPtr&);
    void remoteControllerConnectionStatusCallback(const std_msgs::UInt8::ConstPtr&);
    void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr&);
    void flightStatusCallback(const std_msgs::UInt8::ConstPtr&);
    void heightAboveTakeoffCallback(const std_msgs::Float32ConstPtr&);
    void realTimeKinematicConnectionStatusCallback(const std_msgs::UInt8::ConstPtr&);
    void displayModeSubCallback(const std_msgs::UInt8::ConstPtr&);
    void gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr&);
    void attitudeSubCallback(const geometry_msgs::QuaternionStampedConstPtr&);
    void flightAnomalyCallback(const dji_osdk_ros::FlightAnomaly::ConstPtr&);
    void gimbalAngleCallback(const geometry_msgs::Vector3StampedConstPtr&);
    void waypointV2MissionStateCallback(const dji_osdk_ros::WaypointV2MissionStatePush::ConstPtr&);

    void createDjiSubscribe();

    template<typename DataType>
    void notifySubscribers(const std::type_index&, const DataType) const;

    std::unordered_multimap<std::type_index, std::weak_ptr<Subscriber>> getSubscribers() const;

    void commandPool(std::shared_future<void>);
    void servicePool(std::shared_future<void>);

    ros::NodeHandle _node{};
    ros::Subscriber _gpsHealthRosSubscriber{};
    ros::Subscriber _rcConnectionStatusRosSubscriber{};
    ros::Subscriber _batteryStateRosSubscriber{};
    ros::Subscriber _flightStatusRosSubscriber{};
    ros::Subscriber _heightAboveTakeoffRosSubscriber{};
    ros::Subscriber _rtkConnectionStatusRosSubscriber{};
    ros::Subscriber _displayModeRosSubscriber{};
    ros::Subscriber _gpsPositionRosSubscriber{};
    ros::Subscriber _attitudeRosSubscriber{};
    ros::Subscriber _flightAnomalyRosSubscriber{};
    ros::Subscriber _gimbalAngleRosSubscriber{};
    ros::Subscriber _waypointV2MissionStateRosSubscriber{};

    std::unordered_multimap<std::type_index, std::weak_ptr<Subscriber>> _subscribers{};

    mutable std::mutex _mutexForSubscribers{};

    std::mutex _mutexForInitialize{};
    bool _wasInitialized = false;

    std::shared_ptr<drone::multithread::queue<std::shared_ptr<Command>>> _commandQueue{};

    ServicePool _servicePool{};

    std::thread _threadServicePool;
    std::thread _threadCommandPool;
    std::atomic_flag _lockServicePool = ATOMIC_FLAG_INIT;
};