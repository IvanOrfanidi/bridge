#include <gtest/gtest.h>

#include <application/handler.hpp>
#include <data/test_subscriber.hpp>

namespace {

TEST(HandlerTests, HandlerAttachSubscriber) {
    Handler& handler = Handler::instance();
    auto testSubscriber = std::make_shared<TestSubscriber>(0);

    handler.attachSubscriber(testSubscriber);
    const std::vector testData{0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0};

    for (const auto& expectation : testData) {
        handler.testSubscriberCallback(expectation);

        EXPECT_EQ(std::any_cast<int>(testSubscriber->getRawData()), expectation);
    }
}

TEST(HandlerTests, HandlerDetachSubscriber) {
    Handler& handler = Handler::instance();
    auto testSubscriberDetached = std::make_shared<TestSubscriber>(0);
    auto testSubscriberAttached = std::make_shared<TestSubscriber>(0);

    handler.attachSubscriber(testSubscriberDetached);
    handler.attachSubscriber(testSubscriberAttached);

    constexpr int EXPECTATION = 1;
    const std::vector testData{0, 2, 3, 4, 5, 4, 3, 2, 0};

    handler.testSubscriberCallback(EXPECTATION);
    handler.detachSubscriber(testSubscriberDetached);

    for (const auto& testValue : testData) {
        handler.testSubscriberCallback(testValue);

        EXPECT_EQ(std::any_cast<int>(testSubscriberDetached->getRawData()), EXPECTATION);
        EXPECT_NE(std::any_cast<int>(testSubscriberDetached->getRawData()), testValue);
        EXPECT_NE(std::any_cast<int>(testSubscriberAttached->getRawData()), EXPECTATION);
        EXPECT_EQ(std::any_cast<int>(testSubscriberAttached->getRawData()), testValue);
    }
}

} // namespace