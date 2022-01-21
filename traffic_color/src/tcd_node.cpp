#include <ros/ros.h>
#include "Tcd.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "traffic_color");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  traffic_color::Tcd node(n, pn);

  // Spin and process callbacks
  ros::spin();
}