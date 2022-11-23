#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/services/get_single_battery_dynamic_info_service.hpp>

class GetSingleBatteryDynamicInfoCommunication
  : public GetSingleBatteryDynamicInfoService
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/get-single-battery-dynamic-info";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        const auto data = GetSingleBatteryDynamicInfoService::getData();
        nlohmann::json batteryInfo;
        batteryInfo["battery_index"] = data.info.batteryIndex;
        batteryInfo["current_voltage"] = data.info.currentVoltage;
        batteryInfo["current_electric"] = data.info.currentElectric;
        batteryInfo["full_capacity"] = data.info.fullCapacity;
        batteryInfo["remained_capacity"] = data.info.remainedCapacity;
        batteryInfo["battery_temperature"] = data.info.batteryTemperature;
        batteryInfo["cell_count"] = data.info.cellCount;
        batteryInfo["battery_capacity_percent"] = data.info.batteryCapacityPercent;
        batteryInfo["sop"] = data.info.sop;
        json["info"] = batteryInfo;

        nlohmann::json batteryState;
        batteryState["cell_break"] = data.state.cellBreak;
        batteryState["self_check_error"] = data.state.selfCheckError;
        batteryState["battery_closed_reason"] = data.state.batteryClosedReason;
        batteryState["bat_soh_state"] = data.state.batSOHState;
        batteryState["max_cycle_limit"] = data.state.maxCycleLimit;
        batteryState["has_cell_break"] = data.state.hasCellBreak;
        batteryState["heat_state"] = data.state.heatState;
        json["state"] = batteryState;

        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};