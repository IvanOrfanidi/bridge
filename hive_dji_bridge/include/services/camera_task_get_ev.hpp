#pragma once

#include <dji_osdk_ros/CameraEV.h>

#include <data/services/camera_task_get_ev_service.hpp>
#include <application/ros_service_client.hpp>
#include <services/service.hpp>

class CameraTaskGetEV
  : public Service
  , public RosServiceClient<dji_osdk_ros::CameraEV> {
public:
    explicit CameraTaskGetEV(const unsigned payloadIndex)
      : RosServiceClient(RosServiceName{"/camera_task_get_EV"}) {
        _data.payloadIndex = payloadIndex;
    }

    std::type_index getResultTypeCallback() override {
        return typeid(CameraTaskGetEvService);
    }

    std::any getData() const override {
        return _data;
    }

    bool update() override;

private:
    CameraTaskGetEvData _data{};
};