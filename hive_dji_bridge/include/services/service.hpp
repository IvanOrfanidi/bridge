#pragma once

#include <any>
#include <string>
#include <typeinfo>
#include <typeindex>

class Service {
public:
    virtual ~Service() = default;

    virtual bool update() = 0;

    virtual std::type_index getResultTypeCallback() = 0;

    virtual std::any getData() const = 0;
};
