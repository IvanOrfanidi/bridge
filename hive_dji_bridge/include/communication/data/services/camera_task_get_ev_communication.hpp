#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/services/camera_task_get_ev_service.hpp>

class CameraTaskGetEvCommunication
  : public CameraTaskGetEvService
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/camera-task-get-ev";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        nlohmann::json json;
        const auto data = CameraTaskGetEvService::getData();
        json["payload_index"] = data.payloadIndex;
        json["exposure_compensation"] = data.exposureCompensation;
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};