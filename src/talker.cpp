/**
 * @file talker.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Talker/publisher node
 * @version 0.1
 * @date 2021-10-30
 * 
 * @copyright Copyright (c) 2021. All rights reserved.
 * This project is released under the MIT Public License.
 */

#include <string>
#include <algorithm>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/SetOutputString.h"

std::string output{};

/**
 * @brief Resets the output string object
 * @result Callback function for set_output_string service
 * 
 * @param req 
 * @param resp 
 * @return boolean 
 */
bool set_output_string(beginner_tutorials::SetOutputString::Request &req,
                       beginner_tutorials::SetOutputString::Response &resp) {
  output = std::string(req.msg);
  if (*(output.end()-1) != ' ') output += std::string(" ");
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
  auto change_str_service = n.advertiseService(
    "set_output_string", set_output_string);

  ros::Duration delay(1/1);
  ros::Time begin = ros::Time::now();

  // Set output to the launch file input
  if (argc < 2) ROS_FATAL_STREAM("Did not receive string input to main.");
  output = std::string(argv[1]);

  // Add a space to input if it's not the last char
  if (*(output.end()-1) != ' ') output += std::string(" ");
  ROS_DEBUG_STREAM("Talker node initialized with output string: "
    << output << "\".");

  int count = 0;
  while (ros::ok()) {
    auto current = ros::Time::now();

    // Send out the message after some delay. Cannot delay between
    // iterations because node is now a server as well as a publisher
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
