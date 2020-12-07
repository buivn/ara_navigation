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
serial::Serial serialWheel1;
serial::Serial serialWheel2;
serial::Serial serialUpDown1;
serial::Serial serialUpDown2;
#include <map>
using namespace std;
double speed_robot=20;
// Init variables

char key(' ');

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
    else if(serialport==3) serialUpDown1.write(data);
    //else if(serialport==4) serialUpDown2.write(data);
}
void move_lui()
{
  sendData(1,"L",speed_robot);
  sendData(1,"R",speed_robot);
}
void move_toi()
{
  sendData(1,"L",-speed_robot);
  sendData(1,"R",-speed_robot);
}
void move_phai()
{
  sendData(1,"L",-speed_robot);
  sendData(1,"R",speed_robot);
}
void move_trai()
{
  sendData(1,"L",speed_robot);
  sendData(1,"R",-speed_robot);
}
void stop()
{
  sendData(1,"L",0);
  sendData(1,"R",0);
}
int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "Robot_control");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Create Twist message
  geometry_msgs::Twist twist;
  try
    {
        serialUpDown1.setPort("/dev/ttyUSB0");
        serialUpDown1.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialUpDown1.setTimeout(to);
        serialUpDown1.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(serialUpDown1.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    try
    {
        serialWheel1.setPort("/dev/ttyUSB1");
        serialWheel1.setBaudrate(57600);
        serial::Timeout to1 = serial::Timeout::simpleTimeout(1000);
        serialWheel1.setTimeout(to1);
        serialWheel1.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(serialWheel1.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
  while(ros::ok()){

    // Get the pressed key
    key = getch();
    cout<<key<<endl;
    if(key=='1') {sendData(3,"M",70);cout<<"send 70 to updown1"<<endl;}
    if(key=='2') {sendData(3,"M",66);cout<<"send 66 to updown1"<<endl;}
    if(key=='3') {sendData(3,"M",64);cout<<"send 64 to updown1"<<endl;}
    //if(key=='4') {sendData(4,"M",70);cout<<"send 70 to updown2"<<endl;}
    //if(key=='5') {sendData(4,"M",66);cout<<"send 66 to updown2"<<endl;}
    //if(key=='6') {sendData(4,"M",64);cout<<"send 64 to updown2"<<endl;}

    if(key=='+') {cout<<"tang speed"<<endl;speed_robot++;}
    if(key=='-') {cout<<"giam speed"<<endl;speed_robot--;}
    if(key=='o') {move_toi();cout<<"toi"<<endl;speed_robot--;}
    else if(key=='l') {move_lui();cout<<"lui"<<endl;speed_robot--;}
    else if(key=='k') {move_trai();cout<<"trai"<<endl;speed_robot--;}
    else if(key==';') {move_phai();cout<<"phai"<<endl;speed_robot--;}
    else {stop();cout<<"stop"<<endl;speed_robot--;}
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
