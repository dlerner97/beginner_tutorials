/**
 * @file listener.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Listener/subscriber node 
 * @version 0.1
 * @date 2021-10-30
 * 
 * @copyright Copyright (c) 2021. All rights reserved.
 * This project is released under the MIT Public License.
 */

#include "ros/ros.h"
#include "../include/listener_class.hpp"

/**
 * @brief Listens to chatter topic, prints msgs over the topic and changes ouptut
 *        str after 5 seconds
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  Listener listener(&n);

  ros::Time begin = ros::Time::now();
  ros::Duration change_str_dur(5);
  bool changed{false};

  while (ros::ok()) {
    // After 5 seconds, change the string that the publisher casts out
    if (!changed && (ros::Time::now() - begin > change_str_dur)) {
      changed = true;
      listener.send_service_request("Changed message from default. ");
    }

    listener.check_long_chatter_latency();
    ros::spinOnce();
  }

  ROS_INFO_STREAM("Listener node is shutting down.");
  return 0;
}
