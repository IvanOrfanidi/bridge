#pragma once

#include <dji_osdk_ros/GetWholeBatteryInfo.h>

#include <data/services/get_whole_battery_info_service.hpp>
#include <application/ros_service_client.hpp>
#include <services/service.hpp>

class GetWholeBatteryInfo
  : public Service
  , public RosServiceClient<dji_osdk_ros::GetWholeBatteryInfo> {
public:
    GetWholeBatteryInfo()
      : RosServiceClient(RosServiceName{"/get_whole_battery_info"}) {
    }

    std::type_index getResultTypeCallback() override {
        return typeid(GetWholeBatteryInfoService);
    }

    std::any getData() const override {
        return _data;
    }

    bool update() override;

private:
    WholeBatteryInfoData _data{};
};