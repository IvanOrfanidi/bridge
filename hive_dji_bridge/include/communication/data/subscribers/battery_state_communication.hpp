#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/battery_state.hpp>

class BatteryStateCommunication
  : public BatteryState
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/battery-state";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        const auto data = BatteryState::getData();
        nlohmann::json json;
        headerToJson(data.header, json);
        json["voltage"] = data.voltage;
        json["current"] = data.current;
        json["charge"] = data.charge;
        json["capacity"] = data.capacity;
        json["design_capacity"] = data.designCapacity;
        json["percentage"] = data.percentage;
        json["power_supply_status"] = data.powerSupplyStatus;
        json["power_supply_health"] = data.powerSupplyHealth;
        json["power_supply_technology"] = data.powerSupplyTechnology;
        json["present"] = data.present;
        json["location"] = data.location;
        json["serial_number"] = data.serialNumber;
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};