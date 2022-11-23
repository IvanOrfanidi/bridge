#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/services/camera_task_get_aperture_service.hpp>

class CameraTaskGetApertureCommunication
  : public CameraTaskGetApertureService
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/camera-task-set-aperture";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        const auto data = CameraTaskGetApertureService::getData();
        json["payload_index"] = data.payloadIndex;
        json["aperture"] = data.aperture;
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};