#pragma once

#include <boost/lexical_cast.hpp>
#include <chrono>

namespace drone {
namespace time {

std::string getCurrentTimeAndDateInStringFormat() {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    return boost::lexical_cast<std::string>(std::put_time(std::localtime(&time), "%Y-%m-%d %X"));
}

} // namespace time
} // namespace drone