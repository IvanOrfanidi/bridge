#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/attitude.hpp>

class AttitudeCommunication
  : public Attitude
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/attitude";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        const auto data = Attitude::getData();
        nlohmann::json json;
        headerToJson(data.header, json);
        nlohmann::json quaternion;
        quaternion["x"] = data.quaternion.x;
        quaternion["y"] = data.quaternion.y;
        quaternion["z"] = data.quaternion.z;
        quaternion["w"] = data.quaternion.w;
        json["quaternion"] = std::move(quaternion);
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};