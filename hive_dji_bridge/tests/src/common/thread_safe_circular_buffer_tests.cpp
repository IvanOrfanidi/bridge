#include <gtest/gtest.h>

#include <string>
#include <vector>
#include <thread>

#include <common/thread_safe_circular_buffer.hpp>

namespace {

#define WAITING_FOR_BUFFER_TO_FILL_UP \
    while (source.size() != data.size()) {} // Waiting for the circular buffer to fill up

template<class T>
void tryPopFront(drone::multithread::circular_buffer<T>& source, std::vector<T> data) {
    WAITING_FOR_BUFFER_TO_FILL_UP;

    for (const auto& expected : data) {
        T test;
        EXPECT_TRUE(source.try_pop_front(test));
        EXPECT_EQ(test, expected);
    }
}

template<class T>
void tryPopBack(drone::multithread::circular_buffer<T>& source, std::vector<T> data) {
    WAITING_FOR_BUFFER_TO_FILL_UP;

    for (const auto& expected : data) {
        T test;
        EXPECT_TRUE(source.try_pop_back(test));
        EXPECT_EQ(test, expected);
    }
}

template<class T>
void waitAndPopFront(drone::multithread::circular_buffer<T>& source, std::vector<T> data) {
    WAITING_FOR_BUFFER_TO_FILL_UP;

    for (const auto& expected : data) {
        T test;
        source.wait_and_pop_front(test);
        EXPECT_EQ(test, expected);
    }
}

template<class T>
void waitAndPopBack(drone::multithread::circular_buffer<T>& source, std::vector<T> data) {
    WAITING_FOR_BUFFER_TO_FILL_UP;

    for (const auto& expected : data) {
        T test;
        source.wait_and_pop_back(test);
        EXPECT_EQ(test, expected);
    }
}

template<class T>
void pushBack(drone::multithread::circular_buffer<T>& destination, std::vector<T> data) {
    for (const auto& i : data) {
        destination.push_back(i);
    }
}

template<class T>
void pushFront(drone::multithread::circular_buffer<T>& destination, std::vector<T> data) {
    for (const auto& i : data) {
        destination.push_front(i);
    }
}

template<class T>
class FXThreadSafeCircularBufferPushBackAndTryPopFront : public ::testing::TestWithParam<T> {};
TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushBackAndTryPopFront);
TYPED_TEST_P(FXThreadSafeCircularBufferPushBackAndTryPopFront, ThreadSafeCircularBufferPushBackAndTryPopFront) {
    TypeParam item;
    BOOST_ASSERT_MSG(item.test.size() == item.expected.size(),
                     "The size of the text buffer and the expected buffer must be the same");
    BOOST_ASSERT_MSG(item.test.size() == item.expected_invert.size(),
                     "The size of the text buffer and the expected buffer must be the same");

    typedef typename TypeParam::TestType TestType;
    drone::multithread::circular_buffer<TestType> safeCircularBuffer(item.test.size());

    std::vector<std::thread> threads;
    threads.emplace_back(tryPopFront<TestType>, std::ref(safeCircularBuffer), item.test);
    threads.emplace_back(pushBack<TestType>, std::ref(safeCircularBuffer), item.expected);
    for (auto& thr : threads) {
        thr.join();
    }
}
REGISTER_TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushBackAndTryPopFront, ThreadSafeCircularBufferPushBackAndTryPopFront);

template<class T>
class FXThreadSafeCircularBufferPushBackAndWaitAndPopFront : public ::testing::TestWithParam<T> {};
TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushBackAndWaitAndPopFront);
TYPED_TEST_P(FXThreadSafeCircularBufferPushBackAndWaitAndPopFront, ThreadSafeCircularBufferPushBackAndWaitAndPopFront) {
    TypeParam item;
    BOOST_ASSERT_MSG(item.test.size() == item.expected.size(),
                     "The size of the text buffer and the expected buffer must be the same");
    BOOST_ASSERT_MSG(item.test.size() == item.expected_invert.size(),
                     "The size of the text buffer and the expected buffer must be the same");

    typedef typename TypeParam::TestType TestType;
    drone::multithread::circular_buffer<TestType> safeCircularBuffer(item.test.size());

    std::vector<std::thread> threads;
    threads.emplace_back(waitAndPopFront<TestType>, std::ref(safeCircularBuffer), item.test);
    threads.emplace_back(pushBack<TestType>, std::ref(safeCircularBuffer), item.expected);
    for (auto& thr : threads) {
        thr.join();
    }
}
REGISTER_TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushBackAndWaitAndPopFront,
                           ThreadSafeCircularBufferPushBackAndWaitAndPopFront);

template<class T>
class FXThreadSafeCircularBufferPushFrontAndTryPopFront : public ::testing::TestWithParam<T> {};
TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushFrontAndTryPopFront);
TYPED_TEST_P(FXThreadSafeCircularBufferPushFrontAndTryPopFront, ThreadSafeCircularBufferPushFrontAndTryPopFront) {
    TypeParam item;
    BOOST_ASSERT_MSG(item.test.size() == item.expected.size(),
                     "The size of the text buffer and the expected buffer must be the same");
    BOOST_ASSERT_MSG(item.test.size() == item.expected_invert.size(),
                     "The size of the text buffer and the expected buffer must be the same");

    typedef typename TypeParam::TestType TestType;
    drone::multithread::circular_buffer<TestType> safeCircularBuffer(item.test.size());

    std::vector<std::thread> threads;
    threads.emplace_back(tryPopFront<TestType>, std::ref(safeCircularBuffer), item.test);
    threads.emplace_back(pushFront<TestType>, std::ref(safeCircularBuffer), item.expected_invert);
    for (auto& thr : threads) {
        thr.join();
    }
}
REGISTER_TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushFrontAndTryPopFront, ThreadSafeCircularBufferPushFrontAndTryPopFront);

template<class T>
class FXThreadSafeCircularBufferPushFrontAndWaitAndPopFront : public ::testing::TestWithParam<T> {};
TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushFrontAndWaitAndPopFront);
TYPED_TEST_P(FXThreadSafeCircularBufferPushFrontAndWaitAndPopFront, ThreadSafeCircularBufferPushFrontAndWaitAndPopFront) {
    TypeParam item;
    BOOST_ASSERT_MSG(item.test.size() == item.expected.size(),
                     "The size of the text buffer and the expected buffer must be the same");
    BOOST_ASSERT_MSG(item.test.size() == item.expected_invert.size(),
                     "The size of the text buffer and the expected buffer must be the same");

    typedef typename TypeParam::TestType TestType;
    drone::multithread::circular_buffer<TestType> safeCircularBuffer(item.test.size());

    std::vector<std::thread> threads;
    threads.emplace_back(waitAndPopFront<TestType>, std::ref(safeCircularBuffer), item.test);
    threads.emplace_back(pushFront<TestType>, std::ref(safeCircularBuffer), item.expected_invert);
    for (auto& thr : threads) {
        thr.join();
    }
}
REGISTER_TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushFrontAndWaitAndPopFront,
                           ThreadSafeCircularBufferPushFrontAndWaitAndPopFront);

template<class T>
class FXThreadSafeCircularBufferPushFrontAndTryPopBack : public ::testing::TestWithParam<T> {};
TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushFrontAndTryPopBack);
TYPED_TEST_P(FXThreadSafeCircularBufferPushFrontAndTryPopBack, ThreadSafeCircularBufferPushFrontAndTryPopBack) {
    TypeParam item;
    BOOST_ASSERT_MSG(item.test.size() == item.expected.size(),
                     "The size of the text buffer and the expected buffer must be the same");
    BOOST_ASSERT_MSG(item.test.size() == item.expected_invert.size(),
                     "The size of the text buffer and the expected buffer must be the same");

    typedef typename TypeParam::TestType TestType;
    drone::multithread::circular_buffer<TestType> safeCircularBuffer(item.test.size());

    std::vector<std::thread> threads;
    threads.emplace_back(tryPopBack<TestType>, std::ref(safeCircularBuffer), item.test);
    threads.emplace_back(pushFront<TestType>, std::ref(safeCircularBuffer), item.expected);
    for (auto& thr : threads) {
        thr.join();
    }
}
REGISTER_TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushFrontAndTryPopBack, ThreadSafeCircularBufferPushFrontAndTryPopBack);

template<class T>
class FXThreadSafeCircularBufferPushFrontAndWaitAndPopBack : public ::testing::TestWithParam<T> {};
TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushFrontAndWaitAndPopBack);
TYPED_TEST_P(FXThreadSafeCircularBufferPushFrontAndWaitAndPopBack, ThreadSafeCircularBufferPushFrontAndWaitAndPopBack) {
    TypeParam item;
    BOOST_ASSERT_MSG(item.test.size() == item.expected.size(),
                     "The size of the text buffer and the expected buffer must be the same");
    BOOST_ASSERT_MSG(item.test.size() == item.expected_invert.size(),
                     "The size of the text buffer and the expected buffer must be the same");

    typedef typename TypeParam::TestType TestType;
    drone::multithread::circular_buffer<TestType> safeCircularBuffer(item.test.size());

    std::vector<std::thread> threads;
    threads.emplace_back(waitAndPopBack<TestType>, std::ref(safeCircularBuffer), item.test);
    threads.emplace_back(pushFront<TestType>, std::ref(safeCircularBuffer), item.expected);
    for (auto& thr : threads) {
        thr.join();
    }
}
REGISTER_TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushFrontAndWaitAndPopBack,
                           ThreadSafeCircularBufferPushFrontAndWaitAndPopBack);

template<class T>
class FXThreadSafeCircularBufferPushBackAndTryPopBack : public ::testing::TestWithParam<T> {};
TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushBackAndTryPopBack);
TYPED_TEST_P(FXThreadSafeCircularBufferPushBackAndTryPopBack, ThreadSafeCircularBufferPushBackAndTryPopBack) {
    TypeParam item;
    BOOST_ASSERT_MSG(item.test.size() == item.expected.size(),
                     "The size of the text buffer and the expected buffer must be the same");
    BOOST_ASSERT_MSG(item.test.size() == item.expected_invert.size(),
                     "The size of the text buffer and the expected buffer must be the same");

    typedef typename TypeParam::TestType TestType;
    drone::multithread::circular_buffer<TestType> safeCircularBuffer(item.test.size());

    std::vector<std::thread> threads;
    threads.emplace_back(tryPopBack<TestType>, std::ref(safeCircularBuffer), item.test);
    threads.emplace_back(pushBack<TestType>, std::ref(safeCircularBuffer), item.expected_invert);
    for (auto& thr : threads) {
        thr.join();
    }
}
REGISTER_TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushBackAndTryPopBack, ThreadSafeCircularBufferPushBackAndTryPopBack);

template<class T>
class FXThreadSafeCircularBufferPushBackAndWaitAndPopBack : public ::testing::TestWithParam<T> {};
TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushBackAndWaitAndPopBack);
TYPED_TEST_P(FXThreadSafeCircularBufferPushBackAndWaitAndPopBack, ThreadSafeCircularBufferPushBackAndWaitAndPopBack) {
    TypeParam item;
    BOOST_ASSERT_MSG(item.test.size() == item.expected.size(),
                     "The size of the text buffer and the expected buffer must be the same");
    BOOST_ASSERT_MSG(item.test.size() == item.expected_invert.size(),
                     "The size of the text buffer and the expected buffer must be the same");

    typedef typename TypeParam::TestType TestType;
    drone::multithread::circular_buffer<TestType> safeCircularBuffer(item.test.size());

    std::vector<std::thread> threads;
    threads.emplace_back(waitAndPopBack<TestType>, std::ref(safeCircularBuffer), item.test);
    threads.emplace_back(pushBack<TestType>, std::ref(safeCircularBuffer), item.expected_invert);
    for (auto& thr : threads) {
        thr.join();
    }
}
REGISTER_TYPED_TEST_CASE_P(FXThreadSafeCircularBufferPushBackAndWaitAndPopBack,
                           ThreadSafeCircularBufferPushBackAndWaitAndPopBack);

/* DATA TEST */
template<class T>
struct UnsignedTestItem {
    typedef T TestType;
    const std::vector<T> test{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    const std::vector<T> expected{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    const std::vector<T> expected_invert{9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
};

template<class T>
struct SignedTestItem {
    typedef T TestType;
    const std::vector<T> test{0, -1, -2, -3, -4, -5, -6, -7, -8, -9};

    const std::vector<T> expected{0, -1, -2, -3, -4, -5, -6, -7, -8, -9};

    const std::vector<T> expected_invert{-9, -8, -7, -6, -5, -4, -3, -2, -1, 0};
};

template<class T>
struct CharTestItem {
    typedef T TestType;
    const std::vector<T> test{'A', 'B', 'C', 'D', 'E', 'F', 'a', 'b', 'c', 'd', 'e', 'f'};

    const std::vector<T> expected{'A', 'B', 'C', 'D', 'E', 'F', 'a', 'b', 'c', 'd', 'e', 'f'};

    const std::vector<T> expected_invert{'f', 'e', 'd', 'c', 'b', 'a', 'F', 'E', 'D', 'C', 'B', 'A'};
};

template<class T>
struct FloatingPointTestItem {
    typedef T TestType;
    const std::vector<T> test{-9.9, -8.8, -7.7, -6.6, -5.5, -4.4, -3.3, -2.2, -1.1, 0.0};

    const std::vector<T> expected{-9.9, -8.8, -7.7, -6.6, -5.5, -4.4, -3.3, -2.2, -1.1, 0.0};

    const std::vector<T> expected_invert{0, -1.1, -2.2, -3.3, -4.4, -5.5, -6.6, -7.7, -8.8, -9.9};
};

template<class T>
struct StringTestItem {
    typedef T TestType;
    const std::vector<T> test{"A", "B", "C", "D", "E", "F", "G", "H", "I", "K", "L", "M",
                              "N", "O", "P", "Q", "R", "S", "T", "V", "X", "Y", "Z"};

    const std::vector<T> expected{"A", "B", "C", "D", "E", "F", "G", "H", "I", "K", "L", "M",
                                  "N", "O", "P", "Q", "R", "S", "T", "V", "X", "Y", "Z"};

    const std::vector<T> expected_invert{"Z", "Y", "X", "V", "T", "S", "R", "Q", "P", "O", "N", "M",
                                         "L", "K", "I", "H", "G", "F", "E", "D", "C", "B", "A"};
};

/* Test Types */
typedef ::testing::Types<UnsignedTestItem<unsigned>,
                         SignedTestItem<signed>,
                         CharTestItem<char>,
                         FloatingPointTestItem<float>,
                         FloatingPointTestItem<double>,
                         StringTestItem<std::string>>
  ThreadSafeCircularBufferTypesT;
INSTANTIATE_TYPED_TEST_CASE_P(ThreadSafeCircularBuffer,
                              FXThreadSafeCircularBufferPushBackAndTryPopFront,
                              ThreadSafeCircularBufferTypesT);

INSTANTIATE_TYPED_TEST_CASE_P(ThreadSafeCircularBuffer,
                              FXThreadSafeCircularBufferPushBackAndWaitAndPopFront,
                              ThreadSafeCircularBufferTypesT);

INSTANTIATE_TYPED_TEST_CASE_P(ThreadSafeCircularBuffer,
                              FXThreadSafeCircularBufferPushFrontAndTryPopFront,
                              ThreadSafeCircularBufferTypesT);

INSTANTIATE_TYPED_TEST_CASE_P(ThreadSafeCircularBuffer,
                              FXThreadSafeCircularBufferPushFrontAndWaitAndPopFront,
                              ThreadSafeCircularBufferTypesT);

INSTANTIATE_TYPED_TEST_CASE_P(ThreadSafeCircularBuffer,
                              FXThreadSafeCircularBufferPushFrontAndTryPopBack,
                              ThreadSafeCircularBufferTypesT);

INSTANTIATE_TYPED_TEST_CASE_P(ThreadSafeCircularBuffer,
                              FXThreadSafeCircularBufferPushFrontAndWaitAndPopBack,
                              ThreadSafeCircularBufferTypesT);

INSTANTIATE_TYPED_TEST_CASE_P(ThreadSafeCircularBuffer,
                              FXThreadSafeCircularBufferPushBackAndTryPopBack,
                              ThreadSafeCircularBufferTypesT);

INSTANTIATE_TYPED_TEST_CASE_P(ThreadSafeCircularBuffer,
                              FXThreadSafeCircularBufferPushBackAndWaitAndPopBack,
                              ThreadSafeCircularBufferTypesT);

} // namespace