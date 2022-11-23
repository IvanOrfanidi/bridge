#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/rc_connection_status.hpp>

class RemoteControllerCommunication
  : public RemoteController
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/remote-controller";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        json["rc_status"] = RemoteController::getData();
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};