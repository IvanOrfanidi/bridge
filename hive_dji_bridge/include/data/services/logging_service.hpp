#pragma once

#include <data/subscriber.hpp>
#include <application/logging.hpp>

class LoggingService : public Subscriber {
public:
    static constexpr uint16_t DEFAULT_DATA_BUFFER_SIZE = 64;
    LoggingService()
      : Subscriber(DEFAULT_DATA_BUFFER_SIZE) {
    }

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(LoggingService);
    }

    Logging::Data getData() const {
        const auto data = std::any_cast<Logging::Data>(getRawData());
        return data;
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const LoggingService& loggingService) {
    const auto data = loggingService.getData();
    stream << std::make_tuple(data);
    return stream;
}