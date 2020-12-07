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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>


std::vector<Eigen::VectorXd> list_trajectories;

void trajectoriesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  Eigen::VectorXd jointstate;
  for (int i=0; i<6;i++)
  {
    jointstate(i) = msg->position[i];
  }
  std::cout << jointstate << std::endl;
  list_trajectories.push_back(jointstate);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_processing");
  ros::NodeHandle node_handle("~");
  // ros::AsyncSpinner spinner(1);
  
  ros::Rate loop_rate(4);

  ros::Subscriber tra_sub = node_handle.subscribe("/move_group/fake_controller_joint_state", 50, trajectoriesCallback);


  ros::Publisher jp_pub = node_handle.advertise<std_msgs::Float64MultiArray>("jointState_worming", 1);
  std_msgs::Float64MultiArray jointPosition; 
  // Eigen::VectorXf oneTraj(6);
  std::vector<double> oneTraj = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int check =0;
  while(ros::ok())
  {
    // if (not (list_trajectories.size() == 0))
    if (check < 5)
    {
      for (int k=0; k < 15; k++)
        // oneTraj[k] = list_trajectories[0][k];
        oneTraj[k] += 0.04; 

      // list_trajectories.erase(list_trajectories.begin());
      // std::cout << "just checking this function work properly" << std::endl;
      jointPosition.data.clear();
      for (int i = 0; i < 6; i++)
      {
        // std::cout << oneTraj(i) << std::endl;
        
        jointPosition.data.push_back(oneTraj[i]);
      }
      jp_pub.publish(jointPosition);
      check += 1;
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
  

  return 0;
}
