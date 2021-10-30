#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;
    msg.data = std::string("This is Dani's tutorial. TBH, I've done it wayyy too many times. " + std::to_string(count));

    ROS_INFO_STREAM(msg.data);

    chatter_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }

  return 0;
}