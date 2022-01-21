#include "Tldp.hpp"

namespace group5_sim_launch 
{  
   Tldp::Tldp(ros::NodeHandle& n, ros::NodeHandle& pn):
   listener(buffer)
  {
  
      sub_twist =  n.subscribe("/vehicle/twist",1,&Tldp::recvTwist,this);
      sub_color =  n.subscribe("/traffic_light_color",1,&Tldp::recvColor,this);
      sub_traj  =  n.subscribe("/vehicle_traj",1,&Tldp::recvTraj,this);
      pub_ulc_cmd_ = n.advertise<dataspeed_ulc_msgs::UlcCmd>("/vehicle/ulc_cmd", 1);

      timer = n.createTimer(ros::Duration(0.02), &Tldp::timerCallback, this);
      

  }


void Tldp::timerCallback(const ros::TimerEvent& event)
  {
     
    dataspeed_ulc_msgs::UlcCmd ulc_cmd_msg;

    geometry_msgs::TransformStamped transform1;
    geometry_msgs::TransformStamped transform2;
    geometry_msgs::TransformStamped transform3;
    geometry_msgs::TransformStamped transform4;

    try {
      transform1 = buffer.lookupTransform("base_footprint", "road_intersection_0_traffic_light_1", ros::Time(0));
      transform2 = buffer.lookupTransform("base_footprint", "road_intersection_0_traffic_light_2", ros::Time(0));
      transform3 = buffer.lookupTransform("base_footprint", "road_intersection_0_traffic_light_3", ros::Time(0));
      transform4 = buffer.lookupTransform("base_footprint", "road_intersection_0_traffic_light_4", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN_STREAM(ex.what());
      return;
    }
    dist_traffic_light.data = transform1.transform.translation.x;

  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    secs = ros::Time::now().toSec();
    Distance_to_Light = dist_traffic_light.data;
    current_speed = vehicle_speed.data;
    Distance_to_Stop = Distance_to_Light - Stop_Offset_Light; ///This is the actual distance for the deceleration to occur
    Deceleration_delta_T = (((final_speed *final_speed) - (current_speed*current_speed))/(2* Distance_to_Stop))*delta_T;    
    Deceleration_per_Second = (((final_speed *final_speed) - (current_speed*current_speed))/(2* Distance_to_Stop));   
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO_STREAM("The vehicle speed:" << current_speed);
    ROS_INFO("Distance to Stop:%.2fm", Distance_to_Stop);
    ROS_INFO("Acceleration Necessary to Stop:%.2f", Deceleration_per_Second);
    ROS_INFO("Approaching Traffic Light: %d", Traffic_Light);  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if (transform1.transform.translation.y < 1 && transform1.transform.translation.y >-1 && transform1.transform.translation.x>0 && std::abs(transform1.transform.rotation.z) >0.98)
    {
      Traffic_Light = 1; 
      dist_traffic_light.data = transform1.transform.translation.x;
    }

    else
    {
      if (std::abs(transform3.transform.rotation.z) > 0.98 && transform3.transform.translation.y < 1 && transform3.transform.translation.y >-1 && transform3.transform.translation.x>0)
      {
        Traffic_Light = 3; 
        dist_traffic_light.data = transform1.transform.translation.x;
      }
    else 
      {
       Traffic_Light = 0;
      }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    cycle_secs = secs - (i*35);

    if (cycle_secs<10)
    {
      Color = Green; //Green
    }
     if ((cycle_secs>=10)&&(cycle_secs<15))
    {
      Color = Yellow; //Yellow
    }
     if ((cycle_secs>=15)&&(cycle_secs<35))
    {
      Color = Red; //Red
    }
    if (cycle_secs>=35)
    {
      i = i + 1;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (Color == Green)
    {
      ROS_INFO("Traffic Light: Green");
    }
     if (Color == Yellow)
    {
      ROS_INFO("Traffic Light: Yellow");
    }
     if (Color == Red)
    {
      ROS_INFO("Traffic Light: Red");
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ulc_cmd_msg = trajectory;
      ulc_cmd_msg.linear_decel = 5;//6.375
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
   
if (Traffic_Light != 0)
{
////////////////////////////  
   if (Color == Green)
   {
     ulc_cmd_msg=trajectory;
     Flag = 0;
   }
////////////////////////////
   if ((Color == Yellow)&&(Deceleration_per_Second > -5.00)&&(Deceleration_per_Second <= -3.00))
   {
     if (Flag == 0)
    { 
      Decel_Speed = current_speed;
      Flag = 1;
    }
      ulc_cmd_msg.linear_velocity = Decel_Speed;
      Decel_Speed = Decel_Speed + Deceleration_delta_T;
   }


   if ((Color == Yellow)&&(Deceleration_per_Second < -5.00))
   {
     ulc_cmd_msg=trajectory;
     Flag = 0;
   }

   if ((Color == Yellow)&&(Deceleration_per_Second > -3.00))
   {
     if (Flag == 0)
    { 
      Decel_Speed = current_speed;
      Flag = 1;
    }
      ulc_cmd_msg.linear_velocity = Decel_Speed;
   }
////////////////////////////
    if ((Color == Red)&&(Deceleration_per_Second > -5.00)) //
    {
      if (Flag == 0)
      { 
        Decel_Speed = current_speed;
        Flag = 1;
      }

      if (Distance_to_Stop > 1.00)
      {
        ulc_cmd_msg.linear_velocity = Decel_Speed;
        Decel_Speed = Decel_Speed + Deceleration_delta_T;
      }

      if (Distance_to_Stop <= 1.00)
      {
        Decel_Speed = 0;
        ulc_cmd_msg.linear_velocity = 0;
      }
     }
    }
////////////////////////////
     pub_ulc_cmd_.publish(ulc_cmd_msg); // new use pub ULC // Publish ULC command
  }




  void Tldp::recvColor(const std_msgs::String color)
  {
    light_color = color;
  }


  void Tldp::recvTwist(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    vehicle_speed.data  = msg ->twist.linear.x;       
  }

  void Tldp::recvTraj(const dataspeed_ulc_msgs::UlcCmdConstPtr& traj)
  {
      trajectory = *traj;
  }
 
}