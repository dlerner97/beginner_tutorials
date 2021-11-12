/**
 * @file listener_class.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Listener/subscriber class executable
 * @version 0.1
 * @date 2021-10-30
 * 
 * @copyright Copyright (c) 2021. All rights reserved.
 * This project is released under the MIT Public License.
 */

#include <string>
#include "../include/listener_class.hpp"
#include "beginner_tutorials/SetOutputString.h"

void Listener::chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("I heard: [" << msg->data << "]");
    _disp_error = true;
    _prev_msg_time = ros::Time::now();
}

void Listener::send_service_request(const std::string&& msg) {
    beginner_tutorials::SetOutputString srv;
    srv.request.msg = msg;
    ROS_DEBUG_STREAM("Changing string to \"" << srv.request.msg << "\"");

    bool resp = _req_set_output_client.call(srv);
    if (!resp)
        ROS_FATAL_STREAM("Service returned false!");
    else {
        ROS_DEBUG_STREAM("Succesfully changed message.");
        ROS_INFO_STREAM(
            "\n==========\n" <<
            "Changed string to \"" <<
            srv.request.msg << "\"" <<
            "\n==========\n");
    }
}

void Listener::check_long_chatter_latency() {
    if (ros::Time::now() - _prev_msg_time > _allowed_latency) {
        ROS_ERROR_STREAM_COND(_disp_error,
            "We have not received a message in >=" <<
            _allowed_latency.toSec() <<
            " s. Check if publisher is running.");
        _disp_error = false;
    }
}