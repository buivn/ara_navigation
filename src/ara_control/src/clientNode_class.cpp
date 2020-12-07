#include "ros/ros.h"
// #include "climb3/magarr_controller.h"
#include "std_srvs/SetBool.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sending_magnet_signal");

  // if (argc != 3)
  // {
  //   ROS_INFO("usage: add_two_ints_client X Y");
  //   return 1;
  // }


  ros::NodeHandle n;
  std::string signal="up";
  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("magarr_controller");
  std_srvs::SetBool srv;
  ros::Rate loop_rate(0.1);
  // srv.request.a = atoll(argv[1]);
  // srv.request.b = atoll(argv[2]);

  while(ros::ok())
  {
    srv.request.data = 60;
    // srv.request.signal = signal;

    if (client.call(srv))
    {
      ROS_INFO("Result: %s \n ", srv.response.message.c_str());
      // srv.request.a = 20;
      // srv.request.b = 10;
      // client.call(srv); 
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
      // srv.request.signal = "down";
      // srv.request.b = 10;
      // client.call(srv); 
      return 1;
    }
    // if (signal =="up") signal="down";
    // else signal = "up";

    ros::spinOnce();
    loop_rate.sleep();
  } 


  return 0;
}