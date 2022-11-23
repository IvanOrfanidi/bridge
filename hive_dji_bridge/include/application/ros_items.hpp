#pragma once

#include <string>

class RosNode {
public:
    explicit RosNode(const std::string_view data)
      : _data(data) {
    }

    std::string data() const noexcept {
        return _data;
    }

private:
    const std::string _data;
};

class RosServiceName {
public:
    explicit RosServiceName(const std::string_view data)
      : _data(data) {
    }

    std::string data() const noexcept {
        return _data;
    }

private:
    const std::string _data;
};

class RosPublisherName {
public:
    explicit RosPublisherName(const std::string_view data)
      : _data(data) {
    }

    std::string data() const noexcept {
        return _data;
    }

private:
    const std::string _data;
};