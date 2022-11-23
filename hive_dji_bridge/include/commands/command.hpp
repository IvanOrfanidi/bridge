#pragma once

#include <string>
#include <memory>
#include <typeinfo>
#include <typeindex>

#include <data/commands/command_result.hpp>

class CommandCode {
public:
    explicit CommandCode(const std::string_view data)
      : _data(data) {
    }

    std::string data() const noexcept {
        return _data;
    }

private:
    const std::string _data;
};

class CommandId {
public:
    explicit CommandId(const std::string_view data)
      : _data(data) {
    }

    std::string data() const noexcept {
        return _data;
    }

private:
    const std::string _data{};
};

class CommandPayload {
public:
    explicit CommandPayload(const std::string_view data)
      : _data(data) {
    }

    std::string data() const noexcept {
        return _data;
    }

private:
    const std::string _data;
};

class Command {
public:
    // Return nullptr if the command is unknown. The Command will not be created.
    static std::shared_ptr<Command> create(const CommandCode&, const CommandId&, const CommandPayload&);

    virtual void execute() = 0;

    virtual std::type_index getResultTypeCallback() {
        return typeid(CommandResult);
    }

    virtual std::any getData() const {
        return _data;
    }

    void errorIncorrectPayload() noexcept {
        _data.errorMessage = "Incorrect Payload";
        _data.result = false;
    }

protected:
    explicit Command(const CommandId& id, const CommandPayload& payload)
      : _payload(payload.data()) {
        _data.id = id.data();
    }

    const std::string _payload;
    CommandResultData _data;
};
