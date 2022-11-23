#pragma once

#include <communication/data/communication_interface.hpp>
#include <data/commands/command_result.hpp>

class CommandResultCommunication
  : public CommandResult
  , public CommunicationInterface {
public:
    TransportData getData() const override {
        TransportData transportData("/commands/result");
        const auto data = CommandResult::getData();
        nlohmann::json json;
        json["id"] = data.id;
        json["status"] = data.result ? "Success" : "Error";
        json["error_message"] = data.errorMessage;
        transportData.setData(std::move(json.dump()));
        return transportData;
    }
};