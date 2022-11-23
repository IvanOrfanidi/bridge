#pragma once

#include <iostream>

class TransportData {
public:
    explicit TransportData(const std::string_view path)
      : _path(path) {
    }

    std::string getPath() const noexcept {
        return _path;
    }

    std::string getData() const noexcept {
        return _data;
    }

    void setData(const std::string_view data) {
        _data = data;
    }

private:
    std::string _path;
    std::string _data{};
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const TransportData& data) {
    stream << "path: \'" << data.getPath() << "\', data: " << data.getData();
    return stream;
}
