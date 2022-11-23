#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/gimbal_angle.hpp>

class GimbalAngleCommunication
  : public GimbalAngle
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/gimbal-angle";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        const auto data = GimbalAngle::getData();
        nlohmann::json json;
        headerToJson(data.header, json);
        nlohmann::json vector;
        vector["x"] = data.vector.x;
        vector["y"] = data.vector.y;
        vector["z"] = data.vector.z;
        json["vector"] = std::move(vector);
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};