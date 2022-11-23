#include <gtest/gtest.h>

#include <common/make_string.hpp>

namespace {

#define TO_STRING(__ARG__) std::string(__ARG__)

TEST(MakeStringTests, EmptyString) {
    const std::string test = drone::utils::make_string();

    EXPECT_TRUE(test.empty());
}

TEST(MakeStringTests, StringToString) {
    const std::string expectation = "Hello World!";

    const std::string test = drone::utils::make_string() << expectation;

    EXPECT_EQ(test, expectation);
}

TEST(MakeStringTests, StringStream) {
    drone::utils::make_string makeString;

    makeString << 3 << 2 << 1;

    EXPECT_EQ(TO_STRING(makeString), "321");
}

TEST(MakeStringTests, StringModifiers) {
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << "PI = " << 3.1415), "PI = 3.1415");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::hex << 16), "10");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::oct << 10), "12");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::boolalpha << true), "true");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::boolalpha << false), "false");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::hex << std::showbase << 16), "0x10");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::fixed << std::setprecision(1) << 10.111), "10.1");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::scientific << std::setprecision(2) << 10.111), "1.01e+01");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::noshowpoint << 10.), "10");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::showpos << 11), "+11");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << -100), "-100");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::uppercase << std::hex << 0xabcd), "ABCD");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::nouppercase << std::hex << 0xABCD), "abcd");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::setw(4) << 1), "   1");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::setw(5) << std::internal << -1), "-   1");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::setw(3) << std::left << -1), "-1 ");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::setw(4) << std::right << -1), "  -1");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << std::setfill('.') << std::setw(4) << "n"), "...n");
}

TEST(MakeStringTests, LongStringModifiers) {
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << "PI = " << 3.14159 << " or ~" << std::setprecision(3) << 3.14159),
              "PI = 3.14159 or ~3.14");
    EXPECT_EQ(TO_STRING(drone::utils::make_string()
                        << std::boolalpha << true << " & " << false << " or " << std::noboolalpha << true << " & " << false),
              "true & false or 1 & 0");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << "hexadecimal " << std::uppercase << std::hex << 0x0A << " in"
                                                    << " decimal "
                                                    << "notation " << std::dec << 0x0A),
              "hexadecimal A in decimal notation 10");
    EXPECT_EQ(TO_STRING(drone::utils::make_string() << "i"
                                                    << " = "
                                                    << "-_/^" << -1),
              "i = -_/^-1");
}

} // namespace