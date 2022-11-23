#pragma once

#include <data/subscriber.hpp>

class RemoteController : public Subscriber {
public:
    enum class ConnectionStatus : uint8_t {
        DISCONNECTED = 0,
        CONNECTED = 1,
    };

    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(RemoteController);
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
StreamType& operator<<(StreamType& stream, const RemoteController& remoteController) {
    const unsigned data = remoteController.getData();
    stream << std::make_tuple(data);
    return stream;
}