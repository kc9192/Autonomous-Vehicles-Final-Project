// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <dataspeed_ulc_msgs/UlcCmd.h>

// Namespace matches ROS package name
namespace group5_sim_launch {

  class Tldp {
    public:
      Tldp(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      
      
      ros::Subscriber sub_color;
      ros::Subscriber sub_twist;
      ros::Subscriber sub_traj;
      ros::Publisher pub_ulc_cmd_;

      ros::Timer timer;
      tf2_ros::TransformListener listener;
      tf2_ros::Buffer buffer;

      void timerCallback(const ros::TimerEvent& event);
      std_msgs::Float64 dist_traffic_light;

      void recvTwist(const geometry_msgs::TwistStampedConstPtr& msg);
      std_msgs::Float64 vehicle_speed;

      void recvColor(const std_msgs::String color);
      std_msgs::String light_color;

      void recvTraj(const dataspeed_ulc_msgs::UlcCmdConstPtr& traj);
      dataspeed_ulc_msgs::UlcCmd trajectory;

      //PAUL
      float Deceleration_delta_T;//new
      float Deceleration_per_Second;//new
      int State;
      float delta_T = 0.02; //At 50Hz
      float final_speed = 0.00;
      float current_speed;

      float Distance_to_Light; //Distance from Car to Traffic Light
      const float Stop_Offset_Light = 20.00; //ask?
      float Distance_to_Stop; 

      float Deceleration_Threshhold = -3.09; //This is the maximum decel a car can handle, based of Website dx.doi.org, If our value is smaller than that -> we ignore light

      float Normal_Speed = 20; //This is the normal speed the car is in
      float Old_Speed; //Here the speed value of the cars goes in once!
      float Decel_Speed = Old_Speed + Deceleration_delta_T;

      int Flag = 0;

      int Color = 1;

      ros::Time time;
      double secs;
      double cycle_secs;
      int i = 0;

      int Green = 1;
      int Yellow = 2;
      int Red = 3;

      int Traffic_Light;

      
  };


}