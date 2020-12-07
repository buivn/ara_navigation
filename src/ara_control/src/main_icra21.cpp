#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atoi */
#include <string>
#include <Eigen/Dense>
#include <map>
using namespace std;

// Init variables



int main(int argc, char** argv)
{
  // Init ROS node
  
  ros::init(argc,  argv, "main_icra21");
  ros::NodeHandle nh("~");
  
  // Init cmd_vel publisher
  
  // ros::ServiceClient client_nextPose = nh.serviceClient<geometry_msgs::Vector3>("/next_pose");
  // geometry_msgs::Vector3 srv_nextPose;
  // //setupserial();
  // ros::Rate loop_rate(1);
  // // int mode=0;
  // bool mobile_mode = true;

  // while(ros::ok())
  // {
  //   Eigen::MatrixXf trajectory1(10,3);

  //   trajectory1(0,0) = 0.1;
  //   trajectory1(0,1) = 0.0;
  //   trajectory1(0,2) = 0.0;

  //   if (mobile_mode)
  //   {      
  //     // moving to the convenient Pose 2
  //     srv_nextPose.request.x = trajectory1(0,0);
  //     srv_nextPose.request.y = trajectory1(0,1);
  //     srv_nextPose.request.z = trajectory1(0,2);
  //     // ask for service
  //     client.call(srv_nextPose);

  //     ros::Duration(10.0).sleep();

  //     // get out of the service
  //     mobile_mode = false;
      
  //   }

  //   ros::spinOnce();
  //   loop_rate.sleep();

  // }
  // ros::spinOnce();

  return 0;
}
