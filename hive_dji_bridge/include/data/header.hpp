#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>
#include <common/utils.hpp>

struct Header {
    uint32_t sequence = 0;
    boost::posix_time::ptime stamp{};
    std::string frameId{};
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const Header& header) {
    stream << drone::utils::to_tuple(header);
    return stream;
}
