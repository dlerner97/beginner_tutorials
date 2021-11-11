/**
 * @file talker.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Talker/publisher class executable
 * @version 0.1
 * @date 2021-10-30
 * 
 * @copyright Copyright (c) 2021. All rights reserved.
 * This project is released under the MIT Public License.
 */

#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/SetOutputString.h"
#include "../include/talker_class.hpp"

bool Talker::set_output_string_callback(beginner_tutorials::SetOutputString::Request &req,
                               beginner_tutorials::SetOutputString::Response &resp) {
    set_output(req.msg);
    ROS_WARN_COND(_output == "", "Output has changed to an empty string.");
    ROS_INFO_STREAM("New output: \"" << _output << "\"");
    resp.set = true;
    return true;
}

void Talker::set_output(std::string msg) {
    // Add a space to input if it's not the last char
    if (*(msg.end()-1) != ' ') msg += std::string(" ");
    _output = msg;
}

void Talker::periodic_publish(bool disp_msg) {
    // Send out the message after some delay. Cannot delay between
    // iterations because node is now a server as well as a publisher
    static int count{0};
    auto current = ros::Time::now();
    if (current - _prev_publish_time > _delay) {
        count++;
        std_msgs::String msg;
        msg.data = _output + std::to_string(count);
        _chatter_pub.publish(msg);
        _prev_publish_time = current;
        ROS_INFO_STREAM_COND(disp_msg, msg.data);
    }
}