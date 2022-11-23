#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/services/get_camera_optical_zoom_factor_service.hpp>

class GetCameraOpticalZoomFactorCommunication
  : public GetCameraOpticalZoomFactorService
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/camera-optical-zoom-factor";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        const auto data = GetCameraOpticalZoomFactorService::getData();
        json["payload_index"] = data.payloadIndex;
        json["factor"] = data.factor;
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};