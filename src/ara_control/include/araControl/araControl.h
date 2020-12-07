#ifndef ARACONTROL_H_
#define ARACONTROL_H_

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include <vector>
#include <map>
#include <unistd.h>
#include <sstream>
#include <stdlib.h>     /* atoi */

#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <serial/serial.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>

// #include "ara_control/DynamixelCommand.h"
#include "ara_control/nextPose_move.h"
#include <actionlib/server/simple_action_server.h>
#include <ara_control/araControlAction.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <tf/transform_broadcaster.h>


// namespace araControl
// {
  class araControlAction
  {
    public:     // public function
      // constructor with a point cloud topic
      araControlAction(std::string name);
      // constructor with a pcd file
      ~araControlAction(void) {}

      // initialize function
      void initPublisher();
      void initSubscriber();
      void initServer();
      void initAction();

      // list of Callbackfunction
      void encoder_callback(const geometry_msgs::Vector3::ConstPtr &en);
      void encoder_callback1(const geometry_msgs::Vector3::ConstPtr &en);
      // void joint5_callback(const std_msgs::Int32ConstPtr &msg);
      void joints_callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg);
      // void jointRotate_Callback(const std_msgs::Float64ConstPtr &msg);
      
      // List of services callback
      // bool mobileControl(ara_control::nextPose_move::Request &req, ara_control::nextPose_move::Response &res);
      
      // action Callbacks
      void goalCB();
      void preemptCB();

      // List of functions
      // void rotateJoint5(int32_t target);
      void directJoint5manipulate();

    private:
      
      ros::NodeHandle nh_; // general ROS NodeHandle - used for pub, sub, advertise ...
      ros::NodeHandle nh_private_; // private ROS NodeHandle - used to get parameter from server
      
      // action server variables
      actionlib::SimpleActionServer<ara_control::araControlAction> as_;
      std::string action_name_;
      // int data_count_, goal_;
      // float sum_, sum_sq_;
      ara_control::araControlGoal goal_;
      ara_control::araControlFeedback feedback_;
      ara_control::araControlResult result_;

      // ROS Publisher
      ros::Publisher wheels_pub_;
      ros::Publisher joint5_pub_;
      // ros::Publisher mc_pub_motorAngles_;
      // ros::Publisher cmd_vel_pub_;
      // ros::Publisher mc_pub_;
      
      // ROS Subscribers
      ros::Subscriber mc_sub_encoder_;
      ros::Subscriber mc_sub_joints_; 
      
      // ROS Service Server
      //ros::ServiceServer mc_ser_nextPose_;
      ros::ServiceClient client_theta5_;
      dynamixel_workbench_msgs::DynamixelCommand srv_theta5_;

      
      // Topic name
      std::string motorAnglesTopic_;

      double target_x_;
      double target_y_;
      double target_phi_;
      
      double theta5_;
      double last_enc_l_;
      double last_enc_r_;
      double last_theta5_;
      double last_phi_;
      double last_x_, last_y_;
      // double x_, y_;
      ros::Time last_time_;
      double theta5List_[];
      

  };

// }
#endif /*  */
