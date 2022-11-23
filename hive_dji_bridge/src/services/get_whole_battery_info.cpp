#include <services/get_whole_battery_info.hpp>
#include <commands/payload.hpp>

bool GetWholeBatteryInfo::update() {
    dji_osdk_ros::GetWholeBatteryInfo wholeBatteryInfo;

    _client.call(wholeBatteryInfo);

    const WholeBatteryInfoData data = {BatteryInfoData{
                                         wholeBatteryInfo.response.battery_whole_info.remainFlyTime,
                                         wholeBatteryInfo.response.battery_whole_info.goHomeNeedTime,
                                         wholeBatteryInfo.response.battery_whole_info.landNeedTime,
                                         wholeBatteryInfo.response.battery_whole_info.goHomeNeedCapacity,
                                         wholeBatteryInfo.response.battery_whole_info.landNeedCapacity,
                                         wholeBatteryInfo.response.battery_whole_info.safeFlyRadius,
                                         wholeBatteryInfo.response.battery_whole_info.capacityConsumeSpeed,
                                         wholeBatteryInfo.response.battery_whole_info.goHomeCountDownState,
                                         wholeBatteryInfo.response.battery_whole_info.gohomeCountDownvalue,
                                         wholeBatteryInfo.response.battery_whole_info.voltage,
                                         wholeBatteryInfo.response.battery_whole_info.batteryCapacityPercentage,
                                         wholeBatteryInfo.response.battery_whole_info.lowBatteryAlarmThreshold,
                                         wholeBatteryInfo.response.battery_whole_info.lowBatteryAlarmEnable,
                                         wholeBatteryInfo.response.battery_whole_info.seriousLowBatteryAlarmThreshold,
                                         wholeBatteryInfo.response.battery_whole_info.seriousLowBatteryAlarmEnable,
                                       },
                                       BatteryStateData{
                                         wholeBatteryInfo.response.battery_whole_info.batteryState.voltageNotSafety,
                                         wholeBatteryInfo.response.battery_whole_info.batteryState.veryLowVoltageAlarm,
                                         wholeBatteryInfo.response.battery_whole_info.batteryState.LowVoltageAlarm,
                                         wholeBatteryInfo.response.battery_whole_info.batteryState.seriousLowCapacityAlarm,
                                         wholeBatteryInfo.response.battery_whole_info.batteryState.LowCapacityAlarm,
                                       }};

    if (data == _data) {
        return false;
    }

    _data = data;
    return true;
}