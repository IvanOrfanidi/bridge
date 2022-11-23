#pragma once

#include <dji_osdk_ros/GetCameraOpticalZoomFactor.h>

#include <data/services/get_camera_optical_zoom_factor_service.hpp>
#include <application/ros_service_client.hpp>
#include <services/service.hpp>

class GetCameraOpticalZoomFactor
  : public Service
  , public RosServiceClient<dji_osdk_ros::GetCameraOpticalZoomFactor> {
public:
    explicit GetCameraOpticalZoomFactor(const unsigned payloadIndex)
      : RosServiceClient(RosServiceName{"/dji_osdk_ros/get_camera_optical_zoom_factor"}) {
        _data.payloadIndex = payloadIndex;
    }

    std::type_index getResultTypeCallback() override {
        return typeid(GetCameraOpticalZoomFactorService);
    }

    std::any getData() const override {
        return _data;
    }

    bool update() override;

private:
    GetCameraOpticalZoomFactorData _data{};
};