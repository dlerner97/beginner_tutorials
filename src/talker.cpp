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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "../include/talker_class.hpp"

/**
 * @brief This talker node periodically publishes a string and acts as a service
 *        server, providing other nodes a method to change the output string
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // Set output to the launch file input
  if (argc < 2) ROS_FATAL_STREAM("Did not receive string input to main.");

  Talker talker(&n, std::string(argv[1]), 10);
  
  geometry_msgs::Twist twist;
  twist.linear.x = 1;
  twist.linear.y = 2;
  twist.linear.z = 3;
  twist.angular.x = 4;
  twist.angular.y = 5;
  twist.angular.z = 6;

  while (ros::ok()) {
    talker.periodic_publish();
    twist.linear.x = static_cast<int>(++twist.linear.x) % 5;
    talker.publish_tf(twist);
    ros::spinOnce();
  }
  return 0;
}
