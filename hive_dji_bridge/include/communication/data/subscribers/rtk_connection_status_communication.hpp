#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/rtk_connection_status.hpp>

class RealTimeKinematicCommunication
  : public RealTimeKinematic
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/real-time-kinematic";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        json["rtc_status"] = RealTimeKinematic::getData();
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};