#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16.h>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atoi */
#include "ara_control/DynamixelCommand.h"
#include <string>
#include <Eigen/Dense>
#include <map>
using namespace std;

serial::Serial serialWheel1;
serial::Serial serialWheel2;
serial::Serial serialUpDown1;
serial::Serial serialUpDown2;
string  wheel1_port="/dev/ttyUSB2";
string  wheel2_port="/dev/ttyUSB4";
string  updown1_port="/dev/ttyUSB1";
string  updown2_port="/dev/ttyUSB3";

// Init variables

char key(' ');
double speed_robot=20;

// mobile configuration
int32_t servo1_val1=0;  // joint 6
int32_t servo1_val2=0;  // joint 5
int32_t servo1_val3=276000; // joint 4
int32_t servo1_val4=-135000;  // joint 3
int32_t servo1_val5=0;      // joint 2
int32_t servo1_val6=85000;  // joint 1

int32_t targetJointPos1=0;
int32_t targetJointPos2=0;
int32_t targetJointPos3=0;
int32_t targetJointPos4=0;
int32_t targetJointPos5=0;
int32_t targetJointPos6=0;

bool change = true;


// send Data to a motor
void sendData(int serialport, string A,double x){
    string Data= string(A+to_string(x)+'?');
    int n = Data.length(); 
    char data[n + 1]; 
    strcpy(data, Data.c_str()); 
    if(serialport==1) serialWheel1.write(data);
    else if(serialport==2) serialWheel2.write(data);
    else if(serialport==3) serialUpDown1.write(data);
    else if(serialport==4) serialUpDown2.write(data);
}

int setupserial() {
    try
    {
        serialUpDown1.setPort(updown1_port);
        serialUpDown1.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialUpDown1.setTimeout(to);
        serialUpDown1.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port updown1 ");
        return -1;
    }

    if(serialUpDown1.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    try
    {
        serialUpDown2.setPort(updown2_port);
        serialUpDown2.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialUpDown2.setTimeout(to);
        serialUpDown2.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port updown2 ");
        return -1;
    }

    if(serialUpDown2.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    try
    {
        serialWheel1.setPort(wheel1_port);
        serialWheel1.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialWheel1.setTimeout(to);
        serialWheel1.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port wheel1_port");
        return -1;
    }

    if(serialWheel1.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    try
    {
        serialWheel2.setPort(wheel2_port);
        serialWheel2.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialWheel2.setTimeout(to);
        serialWheel2.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port wheel2_port");
        return -1;
    }

    if(serialWheel2.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
}

void jointState_wormingCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // Eigen::VectorXd jointstate;
  std::cout << "receiving the joint position - outside the loop" << std::endl;
  for (int i=0; i<6;i++)
  {
    if (i==0) targetJointPos1 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==1) targetJointPos2 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==2) targetJointPos3 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==3) targetJointPos4 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==4) targetJointPos5 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==5) targetJointPos6 = int32_t(msg->data[i]*500000/3.141592653);
    // targetJointPos3 = 5000;
    // targetJointPos4 = -5000;
    std::cout << "receiving the joint position" << std::endl;
  }
  change = true;
}


int main(int argc, char** argv)
{
  // Init ROS node
  
  ros::init(argc, argv, "Robot_turning");
  ros::NodeHandle nh("~");
  
  // Init cmd_vel publisher
  // ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  // geometry_msgs::Twist twist;
  ros::ServiceClient client = nh.serviceClient<ara_control::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  ara_control::DynamixelCommand srv;
  
  ros::ServiceClient client_nextPose = nh.serviceClient<geometry_msgs::Vector3>("/next_pose");
  geometry_msgs::Vector3 srv_nextPose;
  //setupserial();
  ros::Rate loop_rate(1);
  // int mode=0;

  bool convPose_2, turning, leg2_touch, convPose_3, mobile_config, joint5_turning;
  nh.param("convPose_2", convPose_2, false);
  nh.param("turning", turning, false);
  nh.param("mobile_config", mobile_config, true);
  nh.param("joint5_turning", joint5_turning, false);
  int angle;
  //nh.param("angle", angle, 5);

  ros::Subscriber tra_sub = nh.subscribe("/trajectory_processing/jointState_worming", 5, jointState_wormingCallback);
  
  // ros::Publisher jp_pub1 = nh.advertise<std_msgs::UInt16>("/setpoint1", 1);
  // ros::Publisher jp_pub2 = nh.advertise<std_msgs::UInt16>("/setpoint2", 1);

  std_msgs::UInt16 Pos1, Pos2; 

  bool turning_mode;
  nh.param("turning_mode", turning_mode, true);

  while(ros::ok())
  {
    if (turning_mode)
    {      
      int32_t convPos_joint1=0, convPos_joint3=-50000, convPos_joint4=140000;
      int32_t convInter_j1=-150000, convInter_j3=-130000, convInter_j4=272000;
      int32_t convInter1_j3=-165000, convInter1_j4=285000;

      // moving to the convenient Pose 2
      if (convPose_2)
      {
        srv.request.command="";
        srv.request.addr_name="Goal_Position";

        // moving the joint 6 - keep the safe distance between two feet
        srv.request.id=1;
        srv.request.value = -7000;
        client.call(srv);

        // moving the joint 1 the base joint - first
        srv.request.id=6;
        // srv.request.value = convPos_joint1;
        srv.request.value = convPos_joint1;
        client.call(srv);
        ros::Duration(3.0).sleep();

        //moving the joint 3 - the second
        srv.request.id=4;
        // srv.request.value = convInter1_j3;
        // client.call(srv);     

        srv.request.id=4;
        srv.request.value = convPos_joint3;
        client.call(srv);
        ros::Duration(15.0).sleep();

        turning = true;
        convPose_2 = false;
        // ros::Duration(2.0).sleep();
        // mobile_config = true;
      }    

      // make a turn of 180 degree
      if (turning)
      {
        srv.request.id=5;
        int32_t convert1 = 400000;

        srv.request.value = convert1; // joint 1    - clockwise - positive
        client.call(srv);
        ros::Duration(120).sleep(); 

        turning = false;
        // move_1leg = false;       
      }

      // change direction
      if (joint5_turning)
      {
        srv.request.command = "";
        
        srv.request.addr_name = "direction";
        int8_t id =1;
        srv.request.id=id;
        //int32_t convert1 = 15000;
        ROS_INFO("Running inside the change_orientation");

        srv.request.value = 100000; // joint 2 (5)
        client.call(srv);

        ros::Duration(10).sleep(); 

        joint5_turning = false;       
      }
      
      if (mobile_config) //(mobile_config)
      {
        srv.request.command="";
        srv.request.addr_name="Goal_Position";
        ROS_INFO("Running inside the mobile configution");
        // moving the joint 6 - keep the safe distance between two feet
        //srv.request.id=1;
        srv.request.id=2;
        srv.request.value =0;
        //srv.request.value = servo1_val1;
        client.call(srv);

        // moving the joint 1 the base joint - first
        //srv.request.id=6;
        // srv.request.value = convPos_joint1;
        //srv.request.value = servo1_val6;
        //client.call(srv);
        //ros::Duration(3.0).sleep();

        //moving the joint 3 - the second
        //srv.request.id=4;
        //srv.request.value = servo1_val4;
        //client.call(srv);
        
        //moving the joint 4 - keep the safe distance between two feet
        //srv.request.id=3;
        // srv.request.value = convPos_joint4;
        //srv.request.value = servo1_val3;
        //client.call(srv);
        //ros::Duration(7.0).sleep();

        turning = false;
        mobile_config =false;
        convPose_2 = false;
      } 


      // switching_controller = true;
      turning_mode = false;
      
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
  // ros::spinOnce();

  return 0;
}
