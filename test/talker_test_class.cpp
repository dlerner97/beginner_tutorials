

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "../include/talker_test_class.hpp"

void TalkerTest::chatterCallback(const std_msgs::String::ConstPtr&) {
    if (_first_msg) {
        _prev_msg_time = ros::Time::now();
        _first_msg = false;
    } else {
        auto current_time = ros::Time::now();
        _sum += (current_time - _prev_msg_time);
        _num_msgs++;
        _prev_msg_time = current_time;
    }  
}

double TalkerTest::get_avg_rate(void) {
    return _num_msgs/_sum.toSec();
}