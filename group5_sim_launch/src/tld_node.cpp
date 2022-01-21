// ROS and node class header file
#include <ros/ros.h>
#include "Tldp.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "group5_sim_launch");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  group5_sim_launch::Tldp node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
