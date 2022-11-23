#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/flight_anomaly.hpp>

class FlightAnomalyCommunication
  : public FlightAnomaly
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/flight-anomaly";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        json["flight_anomaly"] = FlightAnomaly::getData();
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};