#include <services/get_single_battery_dynamic_info.hpp>
#include <commands/payload.hpp>

bool GetSingleBatteryDynamicInfo::update() {
    dji_osdk_ros::GetSingleBatteryDynamicInfo singleBatteryDynamicInfo;
    singleBatteryDynamicInfo.request.batteryIndex = _data.info.batteryIndex;

    _client.call(singleBatteryDynamicInfo);

    const SingleBatteryDynamicInfoData data = {
      SingleBatteryDynamicInfo{
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryIndex,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.currentVoltage,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.currentElectric,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.fullCapacity,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.remainedCapacity,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryTemperature,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.cellCount,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryCapacityPercent,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.SOP,
      },
      SingleBatteryDynamicState{
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryState.cellBreak,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryState.selfCheckError,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryState.batteryClosedReason,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryState.batSOHState,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryState.maxCycleLimit,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryState.hasCellBreak,
        singleBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryState.heatState,
      }};

    if (data == _data) {
        return false;
    }

    _data = data;
    return true;
}