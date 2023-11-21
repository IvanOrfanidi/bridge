#include <boost/assert.hpp>

#include <application/logging.hpp>
#include <application/handler.hpp>
#include <common/make_string.hpp>
#include <data/subscribers/gps_health.hpp>
#include <data/subscribers/rc_connection_status.hpp>
#include <data/subscribers/battery_state.hpp>
#include <data/subscribers/flight_status.hpp>
#include <data/subscribers/height_above_takeoff.hpp>
#include <data/subscribers/rtk_connection_status.hpp>
#include <data/subscribers/display_mode.hpp>
#include <data/subscribers/gps_position.hpp>
#include <data/subscribers/attitude.hpp>
#include <data/subscribers/flight_anomaly.hpp>
#include <data/subscribers/gimbal_angle.hpp>
#include <data/subscribers/waypointV2_mission_state.hpp>
#include <data/subscribers/test_subscriber.hpp>

using namespace std::chrono_literals;

Handler& Handler::instance() {
    static Handler instance;
    instance.initialize();
    return instance;
}

void Handler::initialize() {
    if (_lockInitialized.test_and_set()) {
        return;
    }

    createDjiSubscribe();
}

void Handler::start(std::shared_future<void> future) {
    _threadCommandPool = std::thread(&Handler::commandPool, this, future);
    _threadServicePool = std::thread(&Handler::servicePool, this, future);
}

void Handler::commandPool(std::shared_future<void> future) {
    while (future.wait_for(0ms) == std::future_status::timeout) {
        {
            std::lock_guard<std::mutex> lock(_mutexForCommandQueue);
            while (!_commandQueue->empty()) {
                _lockServicePool.test_and_set(); // Block service pool update
                std::shared_ptr<Command> command;
                _commandQueue->try_pop(command);
                if (command != nullptr) {
                    command->execute();
                    notifyResult(command->getResultTypeCallback(), command->getData());
                }
            }
            _lockServicePool.clear();
        }

        ros::Duration(0.005).sleep();
    }
}

void Handler::servicePool(std::shared_future<void> future) {
    while (future.wait_for(0ms) == std::future_status::timeout) {
        for (const auto& service : _servicePool.services) {
            while (_lockServicePool.test_and_set()) {} // Waiting for commands to complete execution
            while (service->update()) {
                notifyResult(service->getResultTypeCallback(), service->getData());
            }
        }
        ros::Duration(1).sleep();
    }
}

void Handler::attachCommandQueue(const std::shared_ptr<drone::multithread::queue<std::shared_ptr<Command>>> commandQueue) {
    std::lock_guard<std::mutex> lock(_mutexForCommandQueue);
    _commandQueue = std::move(commandQueue);
}

void Handler::attachSubscriber(const std::shared_ptr<Subscriber> subscriber) {
    std::lock_guard<std::mutex> lock(_mutexForSubscribers);
    const auto type = subscriber->getType();
    const auto& [beginSubscriber, endSubscriber] = _subscribers.equal_range(type);
    for (auto it = beginSubscriber; it != endSubscriber; ++it) {
        BOOST_ASSERT_MSG(it->second.lock() != subscriber, "This subscriber is already subscribed");
    }
    _subscribers.insert({type, subscriber});
}

void Handler::detachSubscriber(const std::shared_ptr<Subscriber> subscriber) {
    std::lock_guard<std::mutex> lock(_mutexForSubscribers);
    const auto& [beginSubscriber, endSubscriber] = _subscribers.equal_range(subscriber->getType());
    for (auto it = beginSubscriber; it != endSubscriber; ++it) {
        if (it->second.lock() == subscriber) {
            _subscribers.erase(it);
            return;
        }
    }
}

void Handler::createDjiSubscribe() {
    // clang-format off
    _gpsHealthRosSubscriber = _node.subscribe("/dji_osdk_ros/gps_health", QUEUE_SIZE, &Handler::gpsHealthCallback, this);
    _rcConnectionStatusRosSubscriber = _node.subscribe("/dji_osdk_ros/rc_connection_status", QUEUE_SIZE, &Handler::remoteControllerConnectionStatusCallback, this);
    _batteryStateRosSubscriber = _node.subscribe("/dji_osdk_ros/battery_state", QUEUE_SIZE, &Handler::batteryStateCallback, this);
    _flightStatusRosSubscriber = _node.subscribe("/dji_osdk_ros/flight_status", QUEUE_SIZE, &Handler::flightStatusCallback, this);
    _heightAboveTakeoffRosSubscriber = _node.subscribe("/dji_osdk_ros/height_above_takeoff", QUEUE_SIZE, &Handler::heightAboveTakeoffCallback, this);
    _rtkConnectionStatusRosSubscriber = _node.subscribe("/dji_osdk_ros/rtk_connection_status", QUEUE_SIZE, &Handler::realTimeKinematicConnectionStatusCallback, this);
    _displayModeRosSubscriber = _node.subscribe("/dji_osdk_ros/display_mode", QUEUE_SIZE, &Handler::displayModeSubCallback, this);
    _gpsPositionRosSubscriber = _node.subscribe("/dji_osdk_ros/gps_position", QUEUE_SIZE, &Handler::gpsPositionCallback, this);
    _attitudeRosSubscriber = _node.subscribe("/dji_osdk_ros/attitude", QUEUE_SIZE, &Handler::attitudeSubCallback, this);
    _flightAnomalyRosSubscriber = _node.subscribe("/dji_osdk_ros/flight_anomaly", QUEUE_SIZE, &Handler::flightAnomalyCallback, this);
    _gimbalAngleRosSubscriber = _node.subscribe("/dji_osdk_ros/gimbal_angle", QUEUE_SIZE, &Handler::gimbalAngleCallback, this);
    _waypointV2MissionStateRosSubscriber = _node.subscribe("/dji_osdk_ros/waypointV2_mission_state", QUEUE_SIZE, &Handler::waypointV2MissionStateCallback, this);
    // clang-format on
}

template<typename DataType>
void Handler::notifySubscribers(const std::type_index& type, const DataType data) const {
    auto subscribers = getSubscribers();
    const auto& [beginSubscriber, endSubscriber] = subscribers.equal_range(type);
    for (auto it = beginSubscriber; it != endSubscriber; ++it) {
        const auto subscriber = it->second.lock();
        if (subscriber != nullptr) {
            subscriber->notify(data);
        }
    }
}

std::unordered_multimap<std::type_index, std::weak_ptr<Subscriber>> Handler::getSubscribers() const {
    std::lock_guard<std::mutex> lock(_mutexForSubscribers);
    auto subscribers = _subscribers;
    return subscribers;
}

// Callbacks:
void Handler::gpsHealthCallback(const std_msgs::UInt8::ConstPtr& gpsHealth) {
    notifySubscribers(GpsHealth::getMyselfType(), gpsHealth->data);
}

void Handler::remoteControllerConnectionStatusCallback(const std_msgs::UInt8::ConstPtr& connectionStatus) {
    notifySubscribers(RemoteController::getMyselfType(), connectionStatus->data);
}

void Handler::batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& batteryState) {
    const BatteryState::Data data{{batteryState->header.seq, batteryState->header.stamp.toBoost(), batteryState->header.frame_id},
                                  batteryState->voltage,
                                  batteryState->current,
                                  batteryState->charge,
                                  batteryState->capacity,
                                  batteryState->design_capacity,
                                  batteryState->percentage,
                                  static_cast<BatteryState::PowerSupplyStatus>(batteryState->power_supply_status),
                                  static_cast<BatteryState::PowerSupplyHealth>(batteryState->power_supply_health),
                                  static_cast<BatteryState::PowerSupplyTechnology>(batteryState->power_supply_technology),
                                  batteryState->present,
                                  batteryState->location,
                                  batteryState->serial_number};
    notifySubscribers(BatteryState::getMyselfType(), std::move(data));
}

void Handler::flightStatusCallback(const std_msgs::UInt8::ConstPtr& flightStatus) {
    notifySubscribers(FlightStatus::getMyselfType(), flightStatus->data);
}

void Handler::heightAboveTakeoffCallback(const std_msgs::Float32ConstPtr& height) {
    notifySubscribers(HeightAboveTakeoff::getMyselfType(), height->data);
}

void Handler::realTimeKinematicConnectionStatusCallback(const std_msgs::UInt8::ConstPtr& rtkConnectionStatus) {
    notifySubscribers(RealTimeKinematic::getMyselfType(), rtkConnectionStatus->data);
}

void Handler::displayModeSubCallback(const std_msgs::UInt8::ConstPtr& displayMode) {
    notifySubscribers(DisplayMode::getMyselfType(), displayMode->data);
}

void Handler::gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsData) {
    GpsPosition::Data data;
    data.header = {gpsData->header.seq, gpsData->header.stamp.toBoost(), gpsData->header.frame_id};
    data.navigationSatellite.status = static_cast<GpsPosition::Status>(gpsData->status.status);
    data.navigationSatellite.service = gpsData->status.service;
    data.latitude = gpsData->latitude;
    data.longitude = gpsData->longitude;
    data.altitude = gpsData->altitude;
    BOOST_ASSERT(gpsData->position_covariance.size() == data.positionCovariance.size());
    std::copy(gpsData->position_covariance.begin(), gpsData->position_covariance.end(), data.positionCovariance.begin());
    data.positionCovarianceType = static_cast<GpsPosition::CovarianceType>(gpsData->position_covariance_type);
    notifySubscribers(GpsPosition::getMyselfType(), std::move(data));
}

void Handler::attitudeSubCallback(const geometry_msgs::QuaternionStampedConstPtr& attitudeData) {
    const Attitude::Data data{
      {attitudeData->header.seq, attitudeData->header.stamp.toBoost(), attitudeData->header.frame_id},
      {attitudeData->quaternion.x, attitudeData->quaternion.y, attitudeData->quaternion.z, attitudeData->quaternion.w}};
    notifySubscribers(Attitude::getMyselfType(), std::move(data));
}

void Handler::flightAnomalyCallback(const dji_osdk_ros::FlightAnomaly::ConstPtr& flightAnomaly) {
    notifySubscribers(FlightAnomaly::getMyselfType(), flightAnomaly->data);
}

void Handler::gimbalAngleCallback(const geometry_msgs::Vector3StampedConstPtr& gimbalAngleVector) {
    const GimbalAngle::Data data{
      {gimbalAngleVector->header.seq, gimbalAngleVector->header.stamp.toBoost(), gimbalAngleVector->header.frame_id},
      {gimbalAngleVector->vector.x, gimbalAngleVector->vector.y, gimbalAngleVector->vector.z}};
    notifySubscribers(GimbalAngle::getMyselfType(), std::move(data));
}

void Handler::waypointV2MissionStateCallback(const dji_osdk_ros::WaypointV2MissionStatePush::ConstPtr& waypointV2MissionState) {
    const WaypointV2MissionState::Data data{waypointV2MissionState->commonDataVersion,
                                            waypointV2MissionState->commonDataLen,
                                            waypointV2MissionState->curWaypointIndex,
                                            waypointV2MissionState->state,
                                            waypointV2MissionState->velocity};
    notifySubscribers(WaypointV2MissionState::getMyselfType(), std::move(data));
}

void Handler::testSubscriberCallback(int testData) {
    notifySubscribers(TestSubscriber::getMyselfType(), testData);
}

void Handler::notifyResult(const std::type_index type, const std::any data) const {
    notifySubscribers(type, data);
}
