#pragma once

#include <services/service.hpp>
#include <data/services/serial_number_service.hpp>
#include <application/ros_service_client.hpp>

class SerialNumber : public Service {
public:
    bool update() override {
        if (_serialNumber.empty()) {
            std::string serialNumber;
            ros::NodeHandle node;
            const bool result = node.getParam("/vehicle_node/serial_number", serialNumber);
            if (result && !serialNumber.empty()) {
                Logging::informationMessage(drone::utils::make_string() << "Drone Serial Number: " << serialNumber, __FUNCTION__);
                _serialNumber = serialNumber;
                return true;
            }
        }
        return false;
    }

    std::type_index getResultTypeCallback() override {
        return typeid(SerialNumberService);
    }

    std::any getData() const override {
        return _serialNumber;
    }

private:
    std::string _serialNumber{};
};
