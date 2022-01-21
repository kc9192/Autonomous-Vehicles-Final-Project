// Header file for the class
// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <dynamic_reconfigure/server.h>
#include <traffic_color/traffic_colorConfig.h>

// TF lookup headers
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Image processing and camera geometry headers
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/package.h>

// Namespace matches ROS package name
namespace traffic_color{

  class Tcd {
    public:
      Tcd(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:

      void recvImage(const sensor_msgs::ImageConstPtr& msg);
      void segmentImage(const cv::Mat& raw_img, cv::Mat& bin_img);
      void detectcolor(const cv::Mat& sat_img, const cv::Mat& val_img, cv::Mat& white_bin_img);    

         
      ros::Subscriber sub_image;     
      ros::Publisher pub_color;

      tf2::Transform camera_transform_; 
      bool looked_up_camera_transform_;


      ros::Timer refresh_timer_;

      std::vector<cv::Vec3f> r_circle, g_circle, y_circle, r_circle1;

      dynamic_reconfigure::Server<traffic_colorConfig> srv_;
      void reconfig(traffic_colorConfig& cfg_, uint32_t level);
      traffic_colorConfig config;
      
      //dynamic_reconfigure::Server<HsvExampleConfig> srv_;
      // raw frame
      cv::Mat raw_hsv_img, raw_rgb_img;
      cv::Mat g_threshold, r_threshold, y_threshold, r_threshold1;
      //cv::Mat g_circle, r_circle, y_circle;
      int light;//Default=0, Red=1, Yellow=2, Green=3
      // Input image channels
      cv::Mat hue_channel;
      cv::Mat sat_channel;
      cv::Mat val_channel;

      // Threshold output images
      cv::Mat hue_thres;
      cv::Mat sat_thres;
      cv::Mat val_thres;

      std_msgs::String color;


  };

}