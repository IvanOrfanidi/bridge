#pragma once

#include <string>
#include <nlohmann/json.hpp>

#include <data/header.hpp>

#include <communication/http/transport_data.hpp>

struct CommunicationInterface {
    CommunicationInterface() = default;
    virtual ~CommunicationInterface() = default;

    CommunicationInterface(const CommunicationInterface& other) = default;
    CommunicationInterface& operator=(const CommunicationInterface& other) = default;
    CommunicationInterface(CommunicationInterface&& other) = default;
    CommunicationInterface& operator=(CommunicationInterface&& other) = default;

    virtual TransportData getData() const = 0;

    static void headerToJson(const Header& header, nlohmann::json& json) {
        nlohmann::json data;
        data["sequence"] = header.sequence;
        data["stamp"] = boost::posix_time::to_iso_extended_string(header.stamp);
        data["frame_id"] = header.frameId;
        json["header"] = std::move(data);
    }
};