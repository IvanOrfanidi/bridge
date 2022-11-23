#pragma once

#include <data/subscriber.hpp>

class TestSubscriber : public Subscriber {
public:
    std::type_index getType() override {
        return this->getMyselfType();
    }

    static std::type_index getMyselfType() {
        return typeid(TestSubscriber);
    }
};