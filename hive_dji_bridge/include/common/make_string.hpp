#pragma once

#include <sstream>
#include <memory>

namespace drone {
namespace utils {

struct make_string {
    explicit make_string()
      : _stream(std::make_shared<std::stringstream>()) {
    }

    operator std::string() const {
        return _stream->str();
    }

    explicit operator std::stringstream&() {
        return *_stream;
    }

protected:
    std::shared_ptr<std::stringstream> _stream;
};

template<class T>
make_string operator<<(make_string stream, const T& arg) {
    static_cast<std::stringstream&>(stream) << arg;
    return stream;
}

template<class StreamType>
StreamType& operator<<(StreamType& stream, const make_string& str) {
    stream << static_cast<std::string>(str);
    return stream;
}

} // namespace utils
} // namespace drone