#include <services/logging_info.hpp>
#include <commands/payload.hpp>

bool LoggingInfo::update() {
    return Logging::getLogPool()->try_pop_front(_data);
}