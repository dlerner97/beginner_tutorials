 /**
 * @file talker_test_class.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Test class for talker class declaration
 * @version 0.1
 * @date 2021-10-30
 * 
 * @copyright Copyright (c) 2021. All rights reserved.
 * This project is released under the MIT Public License.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

class TalkerTest { 

    int _num_msgs{0};
    bool _first_msg{true};
    ros::Duration _sum;
    ros::Time _prev_msg_time;
    ros::Subscriber _chatter_sub;

    /**
     * @brief Callback method for the chatter subscriber
     * 
     * @param msg 
     */
    void chatterCallback(const std_msgs::String::ConstPtr& msg);
 
 public:
    TalkerTest(ros::NodeHandle* n) : _sum(0.0) {
        _chatter_sub = n->subscribe("chatter", 1000, &TalkerTest::chatterCallback, this);

        ROS_DEBUG_STREAM("Listener node initialized.");
        _prev_msg_time = ros::Time::now();
    }

    /**
     * @brief Get the avg rate of incoming messages on topic
     * 
     * @return double - avg rate (Hz)
     */
    double get_avg_rate(void);
};