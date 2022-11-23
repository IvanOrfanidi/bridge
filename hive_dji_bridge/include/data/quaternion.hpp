#pragma once

#include <common/utils.hpp>

template<typename TType>
struct Quaternion {
    TType x;
    TType y;
    TType z;
    TType w;
};

template<class StreamType, typename TType>
StreamType& operator<<(StreamType& stream, const Quaternion<TType>& quaternion) {
    stream << drone::utils::to_tuple(quaternion);
    return stream;
}

template<class StreamType>
StreamType& operator<<(StreamType& stream, const Quaternion<uint8_t>& quaternion) {
    stream << std::make_tuple(static_cast<unsigned>(quaternion.x),
                              static_cast<unsigned>(quaternion.y),
                              static_cast<unsigned>(quaternion.z),
                              static_cast<unsigned>(quaternion.w));
    return stream;
}

template<class StreamType>
StreamType& operator<<(StreamType& stream, const Quaternion<int8_t>& quaternion) {
    stream << std::make_tuple(static_cast<int>(quaternion.x),
                              static_cast<int>(quaternion.y),
                              static_cast<int>(quaternion.z),
                              static_cast<int>(quaternion.w));
    return stream;
}