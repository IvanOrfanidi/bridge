#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/subscribers/waypointV2_mission_state.hpp>

class WaypointV2MissionStateCommunication
  : public WaypointV2MissionState
  , public CommunicationInterface {
public:
    static constexpr std::string_view PATH_SEGMENTS = "/state/waypoint-v2-mission-state";

    TransportData getData() const override {
        TransportData transportData(PATH_SEGMENTS.data());
        const auto data = WaypointV2MissionState::getData();
        nlohmann::json json;
        json["common_data_version"] = data.commonDataVersion;
        json["common_data_len"] = data.commonDataLen;
        json["cur_waypoint_index"] = data.curWaypointIndex;
        json["state"] = data.state;
        json["velocity"] = data.velocity;
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};