#pragma once

#include <tuple>
#include <array>

constexpr unsigned long long operator""_minutes(unsigned long long m) {
    return m * 60;
}
constexpr unsigned long long operator""_seconds(unsigned long long s) {
    return s * 1'000;
}

namespace __internal {
template<class Ch, class Tr, class Tuple, std::size_t... Is>
void print_tuple_impl(std::basic_ostream<Ch, Tr>& stream, const Tuple& tuple, std::index_sequence<Is...>) {
    ((stream << (Is == 0 ? "" : ", ") << std::get<Is>(tuple)), ...);
}
} // namespace __internal

template<class Ch, class Tr, class... Args>
auto& operator<<(std::basic_ostream<Ch, Tr>& stream, const std::tuple<Args...>& tuple) {
    stream << '{';
    __internal::print_tuple_impl(stream, tuple, std::index_sequence_for<Args...>{});
    return stream << '}';
}

template<class Ch, class Tr, class T, size_t N>
auto& operator<<(std::basic_ostream<Ch, Tr>& stream, const std::array<T, N>& array) {
    stream << std::tuple_cat(array);
    return stream;
}