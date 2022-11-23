#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/services/camera_task_get_iso_service.hpp>

class CameraTaskGetIsoCommunication
  : public CameraTaskGetIsoService
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/camera-task-get-iso";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        const auto data = CameraTaskGetIsoService::getData();
        json["payload_index"] = data.payloadIndex;
        json["iso_data"] = data.isoData;
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};