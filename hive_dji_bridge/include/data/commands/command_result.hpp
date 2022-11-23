#pragma once

#include <data/subscriber.hpp>
#include <data/commands/command_data.hpp>

class CommandResult : public Subscriber {
public:
    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(CommandResult);
    }

    CommandResultData getData() const {
        return std::any_cast<CommandResultData>(getRawData());
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CommandResult& commandResult) {
    const auto data = commandResult.getData();
    stream << std::make_tuple(data);
    return stream;
}