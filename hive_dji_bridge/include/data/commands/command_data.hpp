#pragma once

#include <string>
#include <iostream>

struct CommandResultData {
    std::string id{};
    bool result = false;
    std::string errorMessage{};
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const CommandResultData& data) {
    stream << "id: {" << data.id << "}, "
           << "result: {" << std::boolalpha << data.result << "}, "
           << "error message: {" << data.errorMessage << '}';
    return stream;
}