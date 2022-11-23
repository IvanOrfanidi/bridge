#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/services/logging_service.hpp>

class LoggingCommunication
  : public LoggingService
  , public CommunicationInterface {
public:
    TransportData getData() const override {
        TransportData transportData("/log");
        nlohmann::json json;

        const auto data = LoggingService::getData();
        nlohmann::json logItem;
        logItem["date_time"] = boost::posix_time::to_iso_extended_string(data.dataTime);
        static const std::map<Logging::Level, std::string> mapOfMsgLevel = {
          {Logging::Level::INFORMATION, "Information"},
          {Logging::Level::WARNING, "Warning"},
          {Logging::Level::ERROR, "Error"},
          {Logging::Level::DEBUG, "Debug"},
        };
        const auto it = mapOfMsgLevel.find(data.level);
        logItem["level"] = (it != mapOfMsgLevel.end()) ? it->second : "UNKNOWN";
        logItem["category"] = '[' + Logging::getNodeName() + "] [" + data.function + ']';
        logItem["message"] = data.message;
        nlohmann::json entries = nlohmann::json::array(); // Log item is sent in array format
        entries.push_back(std::move(logItem));
        json["entries"] = entries;

        json["current_date_time"] = boost::posix_time::to_iso_extended_string(boost::posix_time::second_clock::local_time());

        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};