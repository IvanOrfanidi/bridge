#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/height_above_takeoff.hpp>

class HeightAboveTakeoffCommunication
  : public HeightAboveTakeoff
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/height-above-takeoff";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        json["height_above_takeoff"] = HeightAboveTakeoff::getData();
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};