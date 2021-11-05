/**
 * @file talker.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Talker/publisher node
 * @version 0.1
 * @date 2021-10-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/SetOutputString.h"

std::string output(
  "Dani's Tutorial. Make service request to change string. "
);

bool set_output_string(beginner_tutorials::SetOutputString::Request &req,
                       beginner_tutorials::SetOutputString::Response &resp) {
  output = std::string(req.msg);
  ROS_WARN_COND(output == "", "Output has changed to an empty string.");
  ROS_INFO_STREAM("New output: \"" << output << "\"");
  resp.set = true;
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  auto change_str_service = n.advertiseService("set_output_string", set_output_string);

  ros::Duration delay(1/10);
  ros::Time begin = ros::Time::now();

  ROS_DEBUG_STREAM("Talker node initialized.");

  int count = 0;
  while (ros::ok()) {
    auto current = ros::Time::now();
    if (current - begin > delay) {
      std_msgs::String msg;
      msg.data = output + std::to_string(count);
      ROS_INFO_STREAM_ONCE(msg.data);
      chatter_pub.publish(msg);
      count++;
      begin = current;
    }
    
    ros::spinOnce();
  }
  ROS_DEBUG_STREAM("Talker node shutting down.");
  return 0;
}
