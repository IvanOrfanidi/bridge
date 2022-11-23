#pragma once

#include <common/utils.hpp>

template<typename TType>
struct Vector3 {
    TType x;
    TType y;
    TType z;
};

template<class StreamType, typename T>
StreamType& operator<<(StreamType& stream, const Vector3<T>& vector) {
    stream << drone::utils::to_tuple(vector);
    return stream;
}

template<class StreamType>
StreamType& operator<<(StreamType& stream, const Vector3<uint8_t>& vector) {
    stream << std::make_tuple(static_cast<unsigned>(vector.x), static_cast<unsigned>(vector.y), static_cast<unsigned>(vector.z));
    return stream;
}

template<class StreamType>
StreamType& operator<<(StreamType& stream, const Vector3<int8_t>& vector) {
    stream << std::make_tuple(static_cast<int>(vector.x), static_cast<int>(vector.y), static_cast<int>(vector.z));
    return stream;
}