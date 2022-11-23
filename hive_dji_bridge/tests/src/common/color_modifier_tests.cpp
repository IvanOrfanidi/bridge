#include <gtest/gtest.h>

#include <common/color_modifier.hpp>
#include <common/make_string.hpp>

namespace {

TEST(ColorModifierTests, ColorModifier) {
    // All colors:
    drone::color::Modifier red(drone::color::FG_RED);
    drone::color::Modifier green(drone::color::FG_GREEN);
    drone::color::Modifier yellow(drone::color::FG_YELLOW);
    drone::color::Modifier blue(drone::color::FG_BLUE);
    drone::color::Modifier magenta(drone::color::FG_MAGENTA);
    drone::color::Modifier cyan(drone::color::FG_CYAN);
    drone::color::Modifier white(drone::color::FG_WHITE);
    drone::color::Modifier def(drone::color::FG_DEFAULT);

    // Examples:
    std::cout << red << "This text is red." << def << std::endl;

    const std::string textGreen = drone::utils::make_string() << green << "This text is green." << def;
    std::cout << textGreen << std::endl;

    std::cout << drone::utils::make_string() << blue << "This value " << 3.14159 << " is blue." << def << std::endl;

    const std::string textYellow = drone::utils::make_string()
                                   << yellow << "This value 0x" << std::hex << std::uppercase << 0xABCD << " is yellow." << def;
    std::cout << textYellow << std::endl;

    std::cout << magenta << "This text is magenta " << cyan << "and this text is cyan." << def << std::endl;

    std::cout << white << "This text is white, but the value is red " << red << 100 << def << "." << std::endl;

    std::cout << "This text is default color." << std::endl;

    EXPECT_TRUE(true);
}

} // namespace