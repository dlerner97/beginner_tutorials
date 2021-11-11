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

#include "../include/talker_class.hpp"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // Set output to the launch file input
  if (argc < 2) ROS_FATAL_STREAM("Did not receive string input to main.");

  Talker talker(&n, std::string(argv[1]), 10);

  while (ros::ok()) {
    talker.periodic_publish();
    ros::spinOnce();
  }
  return 0;
}
