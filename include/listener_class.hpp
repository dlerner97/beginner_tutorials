/**
 * @file listener_class.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Listener/subscriber class declaration
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

class Listener {
 private:

    bool _disp_error{true};
    ros::Time _prev_msg_time;
    ros::Duration _allowed_latency;
    ros::Subscriber _chatter_sub;
    ros::ServiceClient _req_set_output_client;

    /**
     * @brief Callback method for the chatter subscriber
     * 
     * @param msg 
     * @result prints message data to ros info
     */
    void chatterCallback(const std_msgs::String::ConstPtr& msg);

 public:
    Listener(ros::NodeHandle* n, double allowed_latency=0.5) :
            _allowed_latency(allowed_latency) {
        _chatter_sub = n->subscribe("chatter", 1000, &Listener::chatterCallback, this);
        _req_set_output_client = n->serviceClient<beginner_tutorials::SetOutputString>(
            "set_output_string");

        ROS_DEBUG_STREAM("Listener node initialized.");
        _prev_msg_time = ros::Time::now();
    }

    /**
     * @brief Sends out a service request to change the output string in the publisher
     * 
     * @param client 
     * @param msg 
     * @result Publisher string will change.
     */
    void send_service_request(const std::string&& msg);

    /**
     * @brief Ensures latency between subsequent messages is less than
     *        a certain rate.
     * 
     * @result Prints warnings to terminal
     */
    void check_long_chatter_latency();
};