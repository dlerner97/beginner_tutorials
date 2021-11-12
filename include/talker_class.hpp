/**
 * @file talker_class.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Talker/publisher class declaration
 * @version 0.1
 * @date 2021-10-30
 * 
 * @copyright Copyright (c) 2021. All rights reserved.
 * This project is released under the MIT Public License.
 */

#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "beginner_tutorials/SetOutputString.h"

class Talker {
 private:

    ros::Duration _delay;
    std::string _output{""};
    ros::Publisher _chatter_pub;
    tf::TransformBroadcaster _br;
    ros::ServiceServer _change_str_service;
    ros::Time _prev_publish_time;

    /**
     * @brief Resets the output string object
     * @result Callback function for set_output_string service
     * 
     * @param req 
     * @param resp 
     * @return boolean 
     */
    bool set_output_string_callback(beginner_tutorials::SetOutputString::Request &req,
                           beginner_tutorials::SetOutputString::Response &resp);

    /**
     * @brief Sets the output string and adds space to end if none exists
     * 
     * @param msg 
     * @result _output var is changed
     */
    void set_output(std::string msg);

 public:
    Talker(ros::NodeHandle* n, std::string def_msg, double hz=10) : _delay(1.0/hz) {
        _chatter_pub = n->advertise<std_msgs::String>("chatter", 1000);
        _change_str_service = n->advertiseService(
            "set_output_string", &Talker::set_output_string_callback, this);
        
        set_output(def_msg);
        ROS_DEBUG_STREAM("Talker node initialized with output string: "
                         << _output << "\".");

        _prev_publish_time = ros::Time::now();
    }

    ~Talker() {
        ROS_DEBUG_STREAM("Talker node shutting down.");
    }

    /**
     * @brief Publishes messages at regular intervals
     * 
     * @param disp_msg - Conditional arg. 
     *                   Whether, we should print to screen 
     * @result Msg is published on chatter topic
     */
    void periodic_publish(bool disp_msg=false);

    /**
     * @brief Publishes given tf to tf broadcaster
     * 
     * @param twist (i.e. translation and rotation btwn frames)
     * @result TF is published to the tf broadcaster
     */
    void publish_tf(const geometry_msgs::Twist& twist);
};