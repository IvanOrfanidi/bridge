#pragma once

#include <dji_osdk_ros/GetSingleBatteryDynamicInfo.h>

#include <data/services/get_single_battery_dynamic_info_service.hpp>
#include <application/ros_service_client.hpp>
#include <services/service.hpp>

class GetSingleBatteryDynamicInfo
  : public Service
  , public RosServiceClient<dji_osdk_ros::GetSingleBatteryDynamicInfo> {
public:
    explicit GetSingleBatteryDynamicInfo(const unsigned batteryIndex)
      : RosServiceClient(RosServiceName{"/get_single_battery_dynamic_info"}) {
        _data.info.batteryIndex = batteryIndex;
    }

    std::type_index getResultTypeCallback() override {
        return typeid(GetSingleBatteryDynamicInfoService);
    }

    std::any getData() const override {
        return _data;
    }

    bool update() override;

private:
    SingleBatteryDynamicInfoData _data{};
};