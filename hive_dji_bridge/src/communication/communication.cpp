#include <boost/assert.hpp>

#include <communication/communication.hpp>
#include <application/logging.hpp>

using namespace std::chrono_literals;

Communication& Communication::instance(const std::string_view url, uint16_t sizeOfCircularBuffer) {
    static Communication instance(url, sizeOfCircularBuffer);
    return instance;
}

void Communication::start(std::shared_future<void> future) {
    _threadData = std::thread(&Communication::sendData, this, future);
    _threadCommand = std::thread(&Communication::receiveCommand, this, future);
}

std::vector<std::shared_ptr<Subscriber>> Communication::getSubscribersData() const {
    std::lock_guard<std::mutex> lock(_mutexForSubscribersData);
    const auto subscribersData = _subscribersData;
    return subscribersData;
}

void Communication::attachCommandQueue(const std::shared_ptr<drone::multithread::queue<std::shared_ptr<Command>>> commandQueue) {
    _commandQueue = commandQueue;
}

void Communication::sendData(std::shared_future<void> future) {
    HttpClient httpClient;
    httpClient.addHeaders(HEADERS_LIST);
    boost::circular_buffer<TransportData> errorBuffer(_dataBuffer.capacity());

    while (future.wait_for(0ms) == std::future_status::timeout) {
        const auto subscribers = getSubscribersData();
        for (const auto& subscriber : subscribers) {
            while (subscriber->haveData()) {
                const auto data = std::dynamic_pointer_cast<CommunicationInterface>(subscriber)->getData();
                const std::string message = drone::utils::make_string() << "JSON data: " << data;
                Logging::informationMessage(message, __FUNCTION__, false);
                _dataBuffer.push_front(std::move(data));
            }
        }

        while (!_dataBuffer.empty()) {
            const TransportData& data = _dataBuffer.front();
            const std::string url = _url + data.getPath();
            const HttpResponse response = httpClient.POST(url.c_str(), data.getData().c_str());
            if (!response.isOK()) {
                errorBuffer.push_back(data);
                Logging::errorMessage(drone::utils::make_string() << "Error HTTP response: " << response, __FUNCTION__);
                Logging::warningMessage(drone::utils::make_string() << "Invalid data: " << data, __FUNCTION__);
            }
            _dataBuffer.pop_front();
        }
        if (!errorBuffer.empty()) {
            _dataBuffer = errorBuffer;
            errorBuffer.clear();
        }

        ros::Duration(0.1).sleep();
    }
}

void Communication::receiveCommand(std::shared_future<void> future) {
    HttpClient httpClient;
    httpClient.addHeaders(HEADERS_LIST);
    const std::string requestsUrl = _url + "/commands/requests";

    while (future.wait_for(0ms) == std::future_status::timeout) {
        const HttpResponse response = httpClient.GET(requestsUrl.c_str());
        if (response.isOK()) {
            parseResponse(response);
        } else {
            Logging::errorMessage(drone::utils::make_string() << "Error HTTP response: " << response, __FUNCTION__);
        }
        ros::Duration(0.1).sleep();
    }
}

void Communication::parseResponse(const HttpResponse& response) {
    if (!response.isOK() || response.isEmpty()) {
        return;
    }

    nlohmann::json data{};
    const std::string rawData = response.getData();
    try {
        data = nlohmann::json::parse(rawData);
    } catch (...) {
        Logging::errorMessage(drone::utils::make_string() << "Invalid JSON: " << rawData, __FUNCTION__);
        return;
    }

    if (!data.is_array()) {
        createCommand(data);
        return;
    }

    for (const auto& command : data) {
        createCommand(command);
    }
}

// Commands factory
void Communication::createCommand(const nlohmann::json& data) {
    BOOST_ASSERT_MSG(_commandQueue != nullptr, "Command queue is not binded");

    if (!data.contains("code") || !data.contains("id") || !data.contains("payload")) {
        Logging::errorMessage("There are no required arguments", __FUNCTION__);
        Logging::warningMessage(drone::utils::make_string() << "Invalid command: " << data, __FUNCTION__);
        return;
    }

    const std::string code = data["code"];
    const std::string id = data["id"];
    const std::string payload = !data.at("payload").is_null() ? data["payload"] : "";

    const auto command = Command::create(CommandCode(code), CommandId(id), CommandPayload(payload));
    if (command != nullptr) { // Return nullptr if the command is unknown.
        _commandQueue->push(command);
    }
}
