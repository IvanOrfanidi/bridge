#include <gtest/gtest.h>

#include <common/utils.hpp>

namespace {

struct AnyStruct {
    int param_1;
    unsigned param_2;
    double param_3;
    char param_4;
    bool param_5;
    std::string param_6;
};

TEST(ToTupleTests, StructToTuple) {
    const AnyStruct struct_1 = {-100, 12'345, 3.14, '%', true, "hello"};
    const AnyStruct struct_2 = {987, 256, -1.23, 'A', false, "world"};

    EXPECT_EQ(drone::utils::to_tuple(struct_1), std::make_tuple(-100, 12'345, 3.14, '%', true, "hello"));
    EXPECT_EQ(drone::utils::to_tuple(struct_2), std::make_tuple(987, 256, -1.23, 'A', false, "world"));
}
} // namespace