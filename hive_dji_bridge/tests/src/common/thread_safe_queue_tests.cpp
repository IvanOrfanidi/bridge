#include <gtest/gtest.h>

#include <string>
#include <vector>
#include <thread>

#include <common/thread_safe_queue.hpp>

namespace {

template<class T>
void first(drone::multithread::queue<T>& source, std::vector<T> data) {
    for (const auto& expected : data) {
        T test;
        source.wait_and_pop(test);
        EXPECT_EQ(test, expected);
    }
}

template<class T>
void second(drone::multithread::queue<T>& destination, std::vector<T> data) {
    for (const auto& i : data) {
        destination.push(i);
    }
}

template<class T>
class FXThreadSafeQueue : public ::testing::TestWithParam<T> {};
TYPED_TEST_CASE_P(FXThreadSafeQueue);
TYPED_TEST_P(FXThreadSafeQueue, ThreadSafeQueue) {
    TypeParam item;

    typedef typename TypeParam::TestType TestType;
    drone::multithread::queue<TestType> safeQueue;

    std::vector<std::thread> threads;
    threads.emplace_back(first<TestType>, std::ref(safeQueue), item.test);
    threads.emplace_back(second<TestType>, std::ref(safeQueue), item.expected);
    for (auto& thr : threads) {
        thr.join();
    }
}
REGISTER_TYPED_TEST_CASE_P(FXThreadSafeQueue, ThreadSafeQueue);

template<class T>
struct UnsignedTestItem {
    typedef T TestType;
    const std::vector<T> test{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
    const std::vector<T> expected{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
};

template<class T>
struct SignedTestItem {
    typedef T TestType;
    const std::vector<T> test{9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9};
    const std::vector<T> expected{9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9};
};

template<class T>
struct CharTestItem {
    typedef T TestType;
    const std::vector<T> test{'A', 'B', 'C', 'D', 'E', 'F', 'a', 'b', 'c', 'd', 'e', 'f'};
    const std::vector<T> expected{'A', 'B', 'C', 'D', 'E', 'F', 'a', 'b', 'c', 'd', 'e', 'f'};
};

template<class T>
struct FloatingPointTestItem {
    typedef T TestType;
    const std::vector<T> test{0, 3.14, 1.23456789, -9.87654321};
    const std::vector<T> expected{0, 3.14, 1.23456789, -9.87654321};
};

template<class T>
struct StringTestItem {
    typedef T TestType;
    const std::vector<T> test{"Hello World", "1234567890 abcd", "", "A B C D E F G H I K L M N O P Q R S T V X Y Z"};
    const std::vector<T> expected{"Hello World", "1234567890 abcd", "", "A B C D E F G H I K L M N O P Q R S T V X Y Z"};
};

/* Test Types */
typedef ::testing::Types<UnsignedTestItem<unsigned>,
                         SignedTestItem<signed>,
                         CharTestItem<char>,
                         FloatingPointTestItem<float>,
                         FloatingPointTestItem<double>,
                         StringTestItem<std::string>>
  ThreadSafeQueueTypesT;
INSTANTIATE_TYPED_TEST_CASE_P(ThreadSafeQueue, FXThreadSafeQueue, ThreadSafeQueueTypesT);

} // namespace