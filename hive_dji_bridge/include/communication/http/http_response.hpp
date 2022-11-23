#pragma once

#include <iostream>
#include <string>

class HttpResponse {
public:
    HttpResponse() = default;
    explicit HttpResponse(bool valid, long status, const std::string_view data)
      : _valid(valid)
      , _status(status)
      , _data(data) {
    }

    bool isOK() const noexcept {
        return _valid && _status == 200;
    }

    bool isEmpty() const noexcept {
        return _data.empty();
    }

    long getStatus() const noexcept {
        return _status;
    }

    std::string getData() const noexcept {
        return _data;
    }

private:
    bool _valid = false;
    long _status = 0;
    std::string _data{};
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const HttpResponse& httpResponse) {
    stream << "Status: {" << httpResponse.getStatus() << "}, Data: {" << httpResponse.getData() << '}';
    return stream;
}
