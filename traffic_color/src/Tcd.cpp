// Header file for the class
#include "Tcd.hpp"

// Namespace matches ROS package name
namespace traffic_color
{  
  
  Tcd::Tcd(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    pub_color = n.advertise<std_msgs::String>("/traffic_light_color", 1);
    sub_image = n.subscribe("traffic_light_camera/image_raw", 1, &Tcd::recvImage, this);

    srv_.setCallback(boost::bind(&Tcd::reconfig, this, _1, _2));
     
  }


  void Tcd::recvImage(const sensor_msgs::ImageConstPtr& msg) 
  {
    
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat raw_rgb_img = cv_ptr->image;

    cv::cvtColor(raw_rgb_img, raw_hsv_img, CV_BGR2HSV);
    
    int detected_r = 0, detected_y = 0, detected_g = 0, total = 0;
    cv::inRange(raw_hsv_img, cv::Scalar(config.r1_h_l, config.r1_s_l, config.r1_v_l), cv::Scalar(config.r1_h_h, config.r1_s_h, config.r1_v_h), r_threshold);
    cv::inRange(raw_hsv_img, cv::Scalar(config.r2_h_l, config.r2_s_l, config.r2_v_l), cv::Scalar(config.r2_h_h, config.r2_s_h, config.r2_v_h), r_threshold1);
    // color: green
    //cv::inRange(raw_hsv_img, cv::Scalar(40, 50, 200), cv::Scalar(90, 255, 255), g_threshold);
    cv::inRange(raw_hsv_img, cv::Scalar(config.g1_h_l, config.g1_s_l, config.g1_v_l), cv::Scalar(config.g1_h_h, config.g1_s_h, config.g1_v_h), g_threshold);
    // color: yellow
    cv::inRange(raw_hsv_img, cv::Scalar(config.y1_h_l, config.y1_s_l, config.y1_v_l), cv::Scalar(config.y1_h_h, config.y1_s_h, config.y1_v_h), y_threshold);

  
    // find red circles
    //int detected_r = 0, detected_y = 0, detected_g = 0;
    cv::HoughCircles(r_threshold, r_circle, cv::HOUGH_GRADIENT,1, 20, 50, 3, 3, 10);
    cv::HoughCircles(r_threshold1, r_circle1, cv::HOUGH_GRADIENT,1, 10, 50, 5, 2, 30);

    // find green circles
    cv::HoughCircles(g_threshold, g_circle, cv::HOUGH_GRADIENT,1, 10, 50, 5, 2, 30);
    // find yellow circles
    cv::HoughCircles(y_threshold, y_circle, cv::HOUGH_GRADIENT,1, 10, 50, 5, 2, 30);

    // detect the red light  cv::Scalar(B, G, R)
    for( size_t i = 0; i < r_circle.size(); i++ )
      { 
        cv::Vec3i c = r_circle[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        circle( raw_rgb_img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        circle( raw_rgb_img, center, radius, cv::Scalar(255,255,255), 3, cv::LINE_AA);
        detected_r++;
      }
    for( size_t i = 0; i < r_circle1.size(); i++ )
      {
        cv::Vec3i c = r_circle1[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        circle( raw_rgb_img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        circle( raw_rgb_img, center, radius, cv::Scalar(255,255,255), 3, cv::LINE_AA);
        detected_r++;
      }
     //detect the yellow lightROS_INFO(" detected_r is %d", detected_r);
     ROS_INFO(" detected_g is %d", detected_g);
     ROS_INFO(" detected_y is %d", detected_y);
    for( size_t i = 0; i < y_circle.size(); i++ )
    {
        cv::Vec3i c = y_circle[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        circle( raw_rgb_img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        circle( raw_rgb_img, center, radius, cv::Scalar(255,255,255), 3, cv::LINE_AA);
        detected_y++;
    }

    // detect the green light
    for( size_t i = 0; i < g_circle.size(); i++ )
      {
        cv::Vec3i c = g_circle[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        circle( raw_rgb_img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        circle( raw_rgb_img, center, radius, cv::Scalar(255,255,255), 3, cv::LINE_AA);
        detected_g++;
      }

   
  

      if (detected_r || detected_y || detected_g)
      {
	    if(detected_r && (! detected_y) && (! detected_g))
      {
		  color.data = "Red";
  	  }
      if(detected_y && (! detected_r) && (! detected_g))
      {
		  color.data = "Yellow";
  	  }
  	  if(detected_g && (! detected_y) && (! detected_r))
      {
		  color.data = "Green";
  	  }
      }

      else {color.data="None";}
   

    
    pub_color.publish(color);
    /*ROS_INFO(" detected_r is %d", detected_r);
    ROS_INFO(" detected_g is %d", detected_g);
    ROS_INFO(" detected_y is %d", detected_y);
    ROS_INFO(" decteted light value is %d", light);
    namedWindow("detected circles", cv::WINDOW_NORMAL);
    cv::resizeWindow("detected circles", 2560, 1440); 
    cv::imshow("detected circles", raw_rgb_img);
    cv::waitKey(1);*/

  }

  void Tcd::reconfig(traffic_colorConfig& cfg_, uint32_t level)
  {
    config = cfg_;
  }


}