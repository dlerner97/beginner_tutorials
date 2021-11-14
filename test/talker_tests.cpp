/**
 * @file talker_tests.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Talker tests
 * @version 0.1
 * @date 2021-10-30
 * 
 * @copyright Copyright (c) 2021. All rights reserved.
 * This project is released under the MIT Public License.
 */

#include <memory>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Twist.h>
#include "../include/talker_class.hpp"
#include "../include/talker_test_class.hpp"

std::shared_ptr<ros::NodeHandle> nh;

TEST(TalkerTests, TestPublishRate) {
    double hz = 10;
    Talker talker(nh.get(), std::string("hello"), hz);
    TalkerTest test(nh.get());
    
    auto begin = ros::Time::now();
    ros::Duration duration(10.0);

    while(ros::ok() && ((ros::Time::now() - begin) < duration)) {
        talker.periodic_publish();
        ros::spinOnce();
    }

    EXPECT_NEAR(test.get_avg_rate(), hz, 0.1);
}

#include <gtest/gtest.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test");
    nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
