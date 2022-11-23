#pragma once

#include <memory>
#include <thread>
#include <future>
#include <boost/circular_buffer.hpp>

#include <communication/http/client.hpp>
#include <communication/http/transport_data.hpp>
#include <communication/data/subscribers/gps_health_communication.hpp>
#include <communication/data/subscribers/rc_connection_status_communication.hpp>
#include <communication/data/subscribers/battery_state_communication.hpp>
#include <communication/data/subscribers/flight_status_communication.hpp>
#include <communication/data/subscribers/height_above_takeoff_communication.hpp>
#include <communication/data/subscribers/rtk_connection_status_communication.hpp>
#include <communication/data/subscribers/display_mode_communication.hpp>
#include <communication/data/subscribers/attitude_communication.hpp>
#include <communication/data/subscribers/flight_anomaly_communication.hpp>
#include <communication/data/subscribers/gimbal_angle_communication.hpp>
#include <communication/data/subscribers/gps_position_communication.hpp>
#include <communication/data/subscribers/waypointV2_mission_state_communication.hpp>

#include <communication/data/services/serial_number_communication.hpp>
#include <communication/data/services/get_camera_optical_zoom_factor_communication.hpp>
#include <communication/data/services/camera_task_get_ev_communication.hpp>
#include <communication/data/services/camera_task_get_iso_communication.hpp>
#include <communication/data/services/camera_task_get_aperture_communication.hpp>
#include <communication/data/services/get_whole_battery_info_communication.hpp>
#include <communication/data/services/get_single_battery_dynamic_info_communication.hpp>
#include <communication/data/services/logging_communication.hpp>

#include <communication/data/commands/command_result_communication.hpp>

#include <common/thread_safe_queue.hpp>
#include <commands/command.hpp>

class Communication {
public:
    static constexpr uint16_t DEFAULT_BUFFER_SIZE = 64;
    static Communication& instance(const std::string_view url, uint16_t sizeOfCircularBuffer = DEFAULT_BUFFER_SIZE);

    void start(std::shared_future<void>);

    std::vector<std::shared_ptr<Subscriber>> getSubscribersData() const;

    void attachCommandQueue(const std::shared_ptr<drone::multithread::queue<std::shared_ptr<Command>>>);

private:
    inline static const std::vector<std::string> HEADERS_LIST{
      "Accept: application/json",
      "Content-Type: application/json",
      "charset: utf-8",
    };

    explicit Communication(const std::string_view url, uint16_t sizeOfCircularBuffer)
      : _url(url)
      , _dataBuffer(sizeOfCircularBuffer) {
    }

    ~Communication() {
        _threadData.join();
        _threadCommand.join();
    }

    void sendData(std::shared_future<void>);

    void receiveCommand(std::shared_future<void>);

    void parseResponse(const HttpResponse&);

    void createCommand(const nlohmann::json&);

    const std::vector<std::shared_ptr<Subscriber>> _subscribersData{
      {std::make_shared<GpsHealthCommunication>()},
      {std::make_shared<RemoteControllerCommunication>()},
      {std::make_shared<BatteryStateCommunication>()},
      {std::make_shared<FlightStatusCommunication>()},
      {std::make_shared<HeightAboveTakeoffCommunication>()},
      {std::make_shared<RealTimeKinematicCommunication>()},
      {std::make_shared<DisplayModeCommunication>()},
      {std::make_shared<AttitudeCommunication>()},
      {std::make_shared<FlightAnomalyCommunication>()},
      {std::make_shared<GimbalAngleCommunication>()},
      {std::make_shared<WaypointV2MissionStateCommunication>()},
      {std::make_shared<GpsPositionCommunication>()},
      {std::make_shared<SerialNumberCommunication>()},
      {std::make_shared<GetCameraOpticalZoomFactorCommunication>()},
      {std::make_shared<CameraTaskGetEvCommunication>()},
      {std::make_shared<CameraTaskGetIsoCommunication>()},
      {std::make_shared<CameraTaskGetApertureCommunication>()},
      {std::make_shared<GetWholeBatteryInfoCommunication>()},
      {std::make_shared<GetSingleBatteryDynamicInfoCommunication>()},
      {std::make_shared<CommandResultCommunication>()},
      {std::make_shared<LoggingCommunication>()},
    };
    mutable std::mutex _mutexForSubscribersData{};

    std::string _url;
    boost::circular_buffer<TransportData> _dataBuffer;

    std::shared_ptr<drone::multithread::queue<std::shared_ptr<Command>>> _commandQueue{};

    std::thread _threadData;
    std::thread _threadCommand;
};