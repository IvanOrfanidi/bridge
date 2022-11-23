#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/display_mode.hpp>

class DisplayModeCommunication
  : public DisplayMode
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/display-mode";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        json["display_mode"] = DisplayMode::getData();
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};