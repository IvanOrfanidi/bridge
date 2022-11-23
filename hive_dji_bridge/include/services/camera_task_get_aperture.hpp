#pragma once

#include <dji_osdk_ros/CameraAperture.h>

#include <data/services/camera_task_get_aperture_service.hpp>
#include <application/ros_service_client.hpp>
#include <services/service.hpp>

class CameraTaskGetAperture
  : public Service
  , public RosServiceClient<dji_osdk_ros::CameraAperture> {
public:
    explicit CameraTaskGetAperture(const unsigned payloadIndex)
      : RosServiceClient(RosServiceName{"/camera_task_get_aperture"}) {
        _data.payloadIndex = payloadIndex;
    }

    std::type_index getResultTypeCallback() override {
        return typeid(CameraTaskGetApertureService);
    }

    std::any getData() const override {
        return _data;
    }

    bool update() override;

private:
    CameraTaskGetApertureData _data{};
};