#pragma once

#include <data/services/logging_service.hpp>
#include <services/service.hpp>

class LoggingInfo : public Service {
public:
    std::type_index getResultTypeCallback() override {
        return typeid(LoggingService);
    }

    std::any getData() const override {
        return _data;
    }

    bool update() override;

private:
    Logging::Data _data{};
};