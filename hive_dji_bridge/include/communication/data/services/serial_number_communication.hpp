#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/services/serial_number_service.hpp>

class SerialNumberCommunication
  : public SerialNumberService
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/serial-number";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        json["value"] = SerialNumberService::getData();
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};