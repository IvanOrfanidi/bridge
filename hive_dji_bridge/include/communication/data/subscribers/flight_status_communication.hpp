#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/flight_status.hpp>

class FlightStatusCommunication
  : public FlightStatus
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/flight-status";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        json["flight_status"] = FlightStatus::getData();
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};