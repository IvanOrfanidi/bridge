#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/services/get_whole_battery_info_service.hpp>

class GetWholeBatteryInfoCommunication
  : public GetWholeBatteryInfoService
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/get-whole-battery-battery-info";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        const auto data = GetWholeBatteryInfoService::getData();
        nlohmann::json batteryInfo;
        batteryInfo["remain_fly_time"] = data.info.remainFlyTime;
        batteryInfo["go_home_need_time"] = data.info.goHomeNeedTime;
        batteryInfo["go_home_need_capacity"] = data.info.goHomeNeedCapacity;
        batteryInfo["land_need_capacity"] = data.info.landNeedCapacity;
        batteryInfo["safe_fly_radius"] = data.info.safeFlyRadius;
        batteryInfo["capacity_consume_speed"] = data.info.capacityConsumeSpeed;
        batteryInfo["go_home_count_down_state"] = data.info.goHomeCountDownState;
        batteryInfo["go_home_count_down_value"] = data.info.gohomeCountDownvalue;
        batteryInfo["voltage"] = data.info.voltage;
        batteryInfo["battery_capacity_percentage"] = data.info.batteryCapacityPercentage;
        batteryInfo["low_battery_alarm_threshold"] = data.info.lowBatteryAlarmThreshold;
        batteryInfo["low_battery_alarm_enable"] = data.info.lowBatteryAlarmEnable;
        batteryInfo["serious_low_battery_alarm_threshold"] = data.info.seriousLowBatteryAlarmThreshold;
        batteryInfo["serious_low_battery_alarm_enable"] = data.info.seriousLowBatteryAlarmEnable;
        json["info"] = batteryInfo;

        nlohmann::json batteryState;
        batteryState["voltage_not_safety"] = data.state.voltageNotSafety;
        batteryState["very_low_voltage_alarm"] = data.state.veryLowVoltageAlarm;
        batteryState["low_voltage_alarm"] = data.state.lowVoltageAlarm;
        batteryState["serious_low_capacity_alarm"] = data.state.seriousLowCapacityAlarm;
        batteryState["low_capacity_alarm"] = data.state.lowCapacityAlarm;
        json["state"] = batteryState;

        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};