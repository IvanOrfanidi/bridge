#pragma once

#include <data/subscriber.hpp>

class RealTimeKinematic : public Subscriber {
public:
    enum class ConnectionStatus : uint8_t {
        DISCONNECTED = 0,
        CONNECTED = 1,
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(RealTimeKinematic);
    }

    uint8_t getData() const {
        return std::any_cast<uint8_t>(getRawData());
    }

    ConnectionStatus getConnectionStatus() const {
        return (getData() == 0) ? ConnectionStatus::DISCONNECTED : ConnectionStatus::CONNECTED;
    }

    std::string getConnectionStatusInString() const {
        return (getConnectionStatus() == ConnectionStatus::DISCONNECTED) ? "disconnected" : "connected";
    }
};

template<class StreamType>
StreamType& operator<<(StreamType& stream, const RealTimeKinematic& realTimeKinematic) {
    const unsigned data = realTimeKinematic.getData();
    stream << std::make_tuple(data);
    return stream;
}