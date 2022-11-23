#pragma once

#include <dji_osdk_ros/CameraISO.h>

#include <data/services/camera_task_get_iso_service.hpp>
#include <application/ros_service_client.hpp>
#include <services/service.hpp>

class CameraTaskGetISO
  : public Service
  , public RosServiceClient<dji_osdk_ros::CameraISO> {
public:
    explicit CameraTaskGetISO(const unsigned payloadIndex)
      : RosServiceClient(RosServiceName{"/camera_task_get_ISO"}) {
        _data.payloadIndex = payloadIndex;
    }

    std::type_index getResultTypeCallback() override {
        return typeid(CameraTaskGetIsoService);
    }

    std::any getData() const override {
        return _data;
    }

    bool update() override;

private:
    CameraTaskGetIsoData _data{};
};