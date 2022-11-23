#include <gtest/gtest.h>
#include <utility>

#include <common/math.hpp>

namespace {

template<typename T>
struct Data {
    T x;
    T y;
};

/** Test structure */
template<typename T>
struct TestItem {
    Data<T> match;
    Data<T> mismatch;
};

template<class T>
class FXIsEqual : public ::testing::TestWithParam<T> {};

TYPED_TEST_CASE_P(FXIsEqual);
TYPED_TEST_P(FXIsEqual, IsEqual) {
    TypeParam test;

    for (const auto& item : test.data) {
        EXPECT_TRUE(drone::math::isEqual(item.match.x, item.match.y));
        EXPECT_FALSE(drone::math::isEqual(item.mismatch.x, item.mismatch.y));
    }
}
REGISTER_TYPED_TEST_CASE_P(FXIsEqual, IsEqual);

template<class T>
struct UnsignedTestItem {
    const std::vector<TestItem<T>> data = {
      {{1, 1}, {1, 2}},
      {{127, 127}, {127, 128}},
      {{255, 255}, {255, 254}},
      {{0, 0}, {0, 1}},
      {{12345, 12345}, {12345, 12344}},
      {{1234567890, 1234567890}, {1234567890, 987654321}},
    };
};

template<class T>
struct SignedTestItemTestItem {
    const std::vector<TestItem<T>> data = {
      {{1, 1}, {1, -1}},
      {{127, 127}, {127, -128}},
      {{255, 255}, {255, 254}},
      {{+0, -0}, {0, -1}},
      {{12345, 12345}, {12345, 12344}},
      {{1234567890, 1234567890}, {1234567890, 987654321}},
    };
};

template<class T>
struct FloatingPointTestItem {
    const std::vector<TestItem<T>> data = {
      {{0, 0}, {0, 0.0001}},
      {{-0, 0}, {0, -0.00001}},
      {{1.1, 1.1}, {1.1, 1.2}},
      {{2.22, 2.22}, {2.22, 2.21}},
      {{1000, 1000}, {1000, 1000.100}},
      {{-1000, -1000}, {-1000, 1000}},
      {{+1.1, 1.1}, {1.1, -1.1}},
      {{22.22, 22.22}, {22.22, 2.221}},
      {{1000.01, 1000.01}, {1000.01, 1000.001}},
      {{-100.00001, -100.00001}, {-100.00001, -100}},
    };
};

/* Test Types */
typedef ::testing::
  Types<UnsignedTestItem<unsigned>, SignedTestItemTestItem<signed>, FloatingPointTestItem<float>, FloatingPointTestItem<double>>
    IsEqualTypesT;
INSTANTIATE_TYPED_TEST_CASE_P(IsEqual, FXIsEqual, IsEqualTypesT);

TEST(IsEqual, IsEqualWithDelta) {
    EXPECT_TRUE(drone::math::isEqual(1234.1, 1234.3, 0.5));
    EXPECT_FALSE(drone::math::isEqual(1234.1, 1234.6, 0.5));

    EXPECT_TRUE(drone::math::isEqual(0.11, 0.12, 0.01));
    EXPECT_FALSE(drone::math::isEqual(0.11, 0.13, 0.01));

    EXPECT_TRUE(drone::math::isEqual(1.234, 1.235, 0.002));
    EXPECT_FALSE(drone::math::isEqual(1.234, 1.232, 0.002));
}

} // namespace