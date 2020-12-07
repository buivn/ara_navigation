/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Hoang-Dung Bui
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Hoang-Dung Bui */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_processing");
  ros::NodeHandle node_handle("~");
  
  ros::Rate loop_rate(0.4);
  
  int  distance1, distance2;
  node_handle.param("distance1", distance1, 52);
  node_handle.param("distance2", distance2, 60);
  
  ros::Publisher jp_pub1 = node_handle.advertise<std_msgs::UInt16>("/setpoint1", 1);
  ros::Publisher jp_pub2 = node_handle.advertise<std_msgs::UInt16>("/setpoint2", 1);
  std_msgs::UInt16 Pos1, Pos2; 

  

  int check =0;
  Pos1.data = distance1;
  Pos2.data = distance2;
  while(ros::ok())
  {
    if (check < 10)
    {
      //Pos.data -= 2;
      jp_pub1.publish(Pos1);
      jp_pub2.publish(Pos2);
      check += 1;
    }
    else if (check == 10)
    {
      Pos1.data = 200;
      jp_pub1.publish(Pos1);
      jp_pub2.publish(Pos1);
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
  

  return 0;
}
