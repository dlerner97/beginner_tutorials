/**
 * @file listener.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Listener/subscriber node 
 * @version 0.1
 * @date 2021-10-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("I heard: [" << msg->data << "]");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  auto sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
