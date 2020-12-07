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
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atoi */
#include "ara_control/DynamixelCommand.h"

serial::Serial serialWheel1;
serial::Serial serialWheel2;
serial::Serial serialUpDown1;
serial::Serial serialUpDown2;
#include <map>
using namespace std;

// Init variables

char key(' ');
double speed_robot=20;
// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}
void sendData(int serialport,string A,double x){
    string Data= string(A+to_string(x)+'?');
    int n = Data.length(); 
    char data[n + 1]; 
    strcpy(data, Data.c_str()); 
    if(serialport==1) serialWheel1.write(data);
    else if(serialport==2) serialWheel2.write(data);
    //else if(serialport==3) serialUpDown1.write(data);
    //else if(serialport==4) serialUpDown2.write(data);
}
void move_lui()
{
  sendData(1,"L",speed_robot);
  sendData(1,"R",speed_robot);
  sendData(2,"L",speed_robot);
  sendData(2,"R",speed_robot);
}
void move_toi()
{
  sendData(1,"L",-speed_robot);
  sendData(1,"R",-speed_robot);
  sendData(2,"L",-speed_robot);
  sendData(2,"R",-speed_robot);
}
void move_phai()
{
  sendData(1,"L",-speed_robot);
  sendData(1,"R",speed_robot);
  sendData(2,"L",-speed_robot);
  sendData(2,"R",speed_robot);
}
void move_trai()
{
  sendData(1,"L",speed_robot);
  sendData(1,"R",-speed_robot);
  sendData(2,"L",speed_robot);
  sendData(2,"R",-speed_robot);
}
void stop()
{
  sendData(1,"L",0);
  sendData(1,"R",0);
  sendData(2,"L",0);
  sendData(2,"R",0);
}
int32_t servo1_val1=-35000;
int32_t servo1_val2=5000;
int32_t servo1_val3=215000;
int32_t servo1_val4=-210000;
int32_t servo1_val5=0;
int32_t servo1_val6=35000;

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "Robot_control");
  ros::NodeHandle nh;
  //ros::ServiceClient client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel_workbench_msgs/DynamixelCommand");
  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  geometry_msgs::Twist twist;
  ros::ServiceClient client = nh.serviceClient<ara_control::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  ara_control::DynamixelCommand srv;
 
  while(ros::ok()){

    // Get the pressed key
    key = getchar();
    cout<<key<<endl;
  
    
    if (key == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }
    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    ros::spinOnce();
  }

  return 0;
}
