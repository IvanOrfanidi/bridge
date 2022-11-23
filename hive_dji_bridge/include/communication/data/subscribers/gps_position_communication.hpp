#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/gps_position.hpp>

class GpsPositionCommunication
  : public GpsPosition
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/gps-position";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        const auto data = GpsPosition::getData();
        nlohmann::json json;
        headerToJson(data.header, json);
        nlohmann::json navigationSatellite;
        navigationSatellite["status"] = data.navigationSatellite.status;
        navigationSatellite["service"] = data.navigationSatellite.service;
        json["navigation_satellite"] = std::move(navigationSatellite);
        json["latitude"] = data.latitude;
        json["longitude"] = data.longitude;
        json["altitude"] = data.altitude;
        json["position_covariance"] = data.positionCovariance;
        json["position_covariance_type"] = data.positionCovarianceType;
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};