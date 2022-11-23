#include <gtest/gtest.h>

#include <common/operators.hpp>
#include <common/make_string.hpp>
#include <common/utils.hpp>

struct AnyStruct {
    int param_1;
    unsigned param_2;
    double param_3;
    char param_4;
    bool param_5;
    std::string param_6;
};

template<class Stream>
Stream& operator<<(Stream& stream, const AnyStruct& anyStruct) {
    stream << drone::utils::to_tuple(anyStruct);
    return stream;
}

namespace {

#define TO_STRING(__ARG__) std::string(__ARG__)

TEST(OperatorTests, OperatorMinutesReturnValue) {
    EXPECT_EQ(1_minutes, 60);
    EXPECT_EQ(5_minutes, 300);
    EXPECT_EQ(10_minutes, 600);
    EXPECT_EQ(20576118_minutes, 1'234'567'080);
    EXPECT_EQ(1234567890_minutes, 74'074'073'400);

    EXPECT_EQ(0_minutes, 0);
    EXPECT_EQ(-0_minutes, 0);

    EXPECT_NE(-1_minutes, 60);
    EXPECT_NE(-5_minutes, 300);
    EXPECT_NE(-10_minutes, 600);
    EXPECT_NE(-20576118_minutes, 1'234'567'080);
    EXPECT_NE(-1234567890_minutes, 74'074'073'400);
}

TEST(OperatorTests, OperatorSecondsReturnValue) {
    EXPECT_EQ(1_seconds, 1'000);
    EXPECT_EQ(5_seconds, 5'000);
    EXPECT_EQ(10_seconds, 10'000);
    EXPECT_EQ(1234567890_seconds, 1'234'567'890'000);

    EXPECT_EQ(0_seconds, 0);
    EXPECT_EQ(-0_seconds, 0);

    EXPECT_NE(-1_seconds, 1'000);
    EXPECT_NE(-5_seconds, 5'000);
    EXPECT_NE(-10_seconds, 10'000);
    EXPECT_NE(-1234567890_seconds, 1'234'567'890'000);
}

TEST(OperatorTests, OperatorStructToOutput) {
    const AnyStruct struct_1 = {-100, 12'345, 3.14, '%', true, "hello"};
    const AnyStruct struct_2 = {987, 256, -1.23, 'A', false, "world"};

    EXPECT_EQ(TO_STRING(drone::utils::make_string() << struct_1), "{-100, 12345, 3.14, %, 1, hello}");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << struct_2), "{987, 256, -1.23, A, 0, world}");
}

} // namespace