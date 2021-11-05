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

#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/SetOutputString.h"

bool disp_error{true};
ros::Time prev_msg_time;

/**
 * @brief Callback function for the chatter subscriber
 * 
 * @param msg 
 * @result prints message data to ros info
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("I heard: [" << msg->data << "]");
  disp_error = true;
  prev_msg_time = ros::Time::now();
}

/**
 * @brief Sends out a service request to change the output string in the publisher
 * 
 * @param client 
 * @param msg 
 * @result Publisher string will change.
 */
void send_service_request(ros::ServiceClient* client, const std::string&& msg) {
    beginner_tutorials::SetOutputString srv;
    srv.request.msg = msg;
    ROS_DEBUG_STREAM("Changing string to \"" << srv.request.msg << "\"");

    bool resp = client->call(srv);
    if (!resp) {
      ROS_FATAL_STREAM("Service returned false!");
    } else {
      ROS_DEBUG_STREAM("Succesfully changed message.");
      ROS_INFO_STREAM(
        "\n==========\n" <<
        "Changed string to \"" <<
        srv.request.msg << "\"" <<
        "\n==========\n");
    }
}

/**
 * @brief Subscriber to the chatter topic and service client to set_output string
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  auto sub = n.subscribe("chatter", 1000, chatterCallback);
  auto client = n.serviceClient<beginner_tutorials::SetOutputString>(
    "set_output_string");

  ROS_DEBUG_STREAM("Listener node initialized.");

  prev_msg_time = ros::Time::now();
  ros::Duration allowed_latency = ros::Duration(0.5);

  ros::Time begin = ros::Time::now();
  ros::Duration change_str_dur(5);
  bool changed{false};

  while (ros::ok()) {
    // After 5 seconds, change the string that the publisher casts out
    if (!changed && (ros::Time::now() - begin > change_str_dur)) {
      changed = true;
      send_service_request(&client, "Changed message from default. ");
    }

    // If latency between subsequent chatter messages is more than
    // some allowed latency, complain.
    if (ros::Time::now() - prev_msg_time > allowed_latency) {
      ROS_ERROR_STREAM_COND(disp_error,
        "We have not received a message in >=" <<
        allowed_latency.toSec() <<
        " s. Check if publisher is running.");
      disp_error = false;
    }

    ros::spinOnce();
  }

  ROS_INFO_STREAM("Listener node is shutting down.");
  return 0;
}
