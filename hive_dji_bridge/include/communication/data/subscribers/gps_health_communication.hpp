#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/gps_health.hpp>

class GpsHealthCommunication
  : public GpsHealth
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/gps-health";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        json["gps_health"] = GpsHealth::getData();
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};