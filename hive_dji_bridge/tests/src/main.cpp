#include <gtest/gtest.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "unit_tests");
    return RUN_ALL_TESTS();
}