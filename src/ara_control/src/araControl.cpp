#include <std_msgs/UInt16.h>
#include <araControl/araControl.h>

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

double L = 0.25; // distance between two feet (m)
double diameter = 0.1, w=0.1; // diamter of wheel (m)
ros::Time time1;
double tickspercount = 600*(235/127);
double pi = 3.14159265;
double TICKS_PER_METER= tickspercount/(pi*diameter);


// namespace araControl
// {
// constructor
araControlAction::araControlAction(std::string name): 
  as_(nh_, name, false),
  // nh_private_(ros::NodeHandle("~")),
  action_name_(name)
{
  target_x_ = 0.0;
  target_y_ = 0.0;
  target_phi_ = 0.0;
  last_enc_l_ = 0.0;
  last_enc_r_ = 0.0;
  last_theta5_ = 0.0;
  last_phi_ = 0.0;
  last_x_ = 0.0;
  last_y_ = 0.0;
  last_time_ = ros::Time::now();
  // initialize other element
  initPublisher();
  initSubscriber();
  initServer();
  // the initilization of Action should be the last one
  initAction();
}

// ~AraControL() {}
//~MobileControL::MobileControL(void)

/********************************************************************************
** Init Functions
********************************************************************************/
void araControlAction::initAction() {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&araControlAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&araControlAction::preemptCB, this));
  as_.start();
  ROS_INFO("in the initAction");
}


void araControlAction::initPublisher() {
  // ros message publisher
  wheels_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/wheels_move", 50);
  joint5_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelState>("/dynamixel_workbench/joint5_control", 10);
}

void araControlAction::initSubscriber() {
  // ros message subscriber
  mc_sub_encoder_ = nh_.subscribe ("encoder", 50, &araControlAction::encoder_callback1, this);
  mc_sub_joints_ = nh_.subscribe ("/dynamixel_workbench/dynamixel_state", 50, &araControlAction::joints_callback, this);
}

void araControlAction::initServer() {
  //mc_ser_nextPose_ = nh_.advertiseService("nextPose_move", &MobileControL::mobileControl, this);
  client_theta5_ = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
}

// get the joint `okokck dang angles from the 
void araControlAction::joints_callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg) {
  // theta5List_.push_back(msg->data);
  theta5_ = msg->dynamixel_state[1].present_position;
  // ROS_INFO("check out the callgmuabacks functions of joint 5");
}

void araControlAction::encoder_callback1(const geometry_msgs::Vector3::ConstPtr& en) {
  ROS_INFO("check out the callbacks functions of encoders");
  if (!as_.isActive())
    return;
  
  double enc_l, enc_r;
  float del_l, del_r;
  enc_l = en->x;
  enc_r = en->y;
  del_l = enc_l - 1000.0;
  del_r = enc_r - 1000.0;
  
  if ((abs(del_l) < 2.0) and (abs(del_r) < 2.0)) {
    ROS_INFO("Reached the intermediate target");
    result_.step_target.x = 1000.0;
    result_.step_target.y = 1000.0;
    result_.step_target.z = 0.0;
    result_.done = true;
    as_.setSucceeded(result_);
  }
  else {
    geometry_msgs::Pose2D wheelsMove;
    wheelsMove.x = del_l;
    wheelsMove.y = del_r;
    wheels_pub_.publish(wheelsMove);
    
    ROS_INFO("robot is moving");
    feedback_.current_pose.x = del_l;
    feedback_.current_pose.y = del_r;
    feedback_.current_pose.z = 0.0;
    as_.publishFeedback(feedback_);
  }
}


void araControlAction::encoder_callback(const geometry_msgs::Vector3::ConstPtr& en) {
  ROS_INFO("check out the callbacks functions of encoders");
  if (!as_.isActive())
    return;
  
  dynamixel_workbench_msgs::DynamixelState joint5;

  ROS_INFO("Get data from encoder: %d", 1);
  double enc_l, enc_r, arc_r, arc_l, x, y, theta5, phi, d, alpha, d_time;
  enc_l = en->x;
  enc_r = en->y;

  arc_l = (enc_l - last_enc_l_)/TICKS_PER_METER;
  arc_r = (enc_r - last_enc_r_)/TICKS_PER_METER;
  last_enc_l_ = enc_l;
  last_enc_r_ = enc_r;
  // determine the current robot pose
  // if the robot go straight (steering angle = 0)
  if (last_theta5_ == 0.0) {
      x = last_x_ + arc_r*cos(last_phi_);
      y = last_y_ + arc_r*sin(last_phi_);
      phi = last_phi_;  
  }
  else {
      d = L/tan(theta5);
      alpha = arc_r*2/diameter;
      x = last_x_ + d*sin(last_phi_) - d*cos(alpha+pi/2-last_phi_);
      y = last_y_ - d*cos(last_phi_) + d*sin(alpha+pi/2-last_phi_);
      phi = last_phi_ - alpha;
  }
  last_x_ = x;
  last_y_ = y;
  last_phi_ = phi;
  time1 = ros::Time::now();
  d_time = (time1 - last_time_).toSec();
  last_time_ = time1;

  // Pure pursuit control law 
  float beta, alpha2, del_x, del_y, del_phi;
  del_y = target_y_ - y;
  del_x = target_x_ - x;
  del_phi = target_phi_ - phi;

  // if reach the destination
  if ((abs(del_x) < 0.02) and (abs(del_y) < 0.02) and (abs(del_phi) < 0.02)) {
    ROS_INFO("Reached the intermediate target");
    result_.step_target.x = x;
    result_.step_target.y = y;
    result_.step_target.z = phi;
    result_.done = true;
    as_.setSucceeded(result_);
  }
  // not get the destination
  else
  {
    // a pure control law
    beta = atan(del_y/del_x);
    alpha2 = phi + beta;
    theta5 = atan(2*L*sin(alpha2)/sqrt(pow(del_x,2) + pow(del_y,2)));
    last_theta5_ = theta5;
  
    // Stanley control law

    // send the data to the joint 5, and wheel controller
    double r1, r2, ratio1lr, ratio2lr;
    // check division by 0  
    if ((theta5 > 0.002) or (theta5 < -0.002)) {
      if ((theta5 < 0.55) and (theta5 > -0.55))
      {
        r1 = L/tan(theta5);
        r2 = L/sin(theta5);
        ratio1lr = (r1 + w)/(r1 - w);
        ratio2lr = (r2 + w)/(r2 - w);
      }
      else {
        ROS_INFO("theta 5 is out of range");
      }
    }
    else {
      ratio1lr = 1.0;
      ratio2lr = 1.0;
    }

    // send command to rotate the joint 5 - need to change to topics
    joint5.id = 2;  
    joint5.name = "Goal_Position";
    joint5.present_position = theta5; // target need to go to
    // joint5_pub_.publish(joint5);
    // publish the the whelle controller
    geometry_msgs::Pose2D wheelsMove;
    wheelsMove.x = ratio1lr;
    wheelsMove.y = ratio2lr;
    // wheels_pub_.publish(wheelsMove);
    
    ROS_INFO("robot is moving");
    feedback_.current_pose.x = x;
    feedback_.current_pose.y = y;
    feedback_.current_pose.z = phi;
    as_.publishFeedback(feedback_);
  }
}

void araControlAction::goalCB() {
  ROS_INFO("The goal arrived");
  geometry_msgs::Vector3 target;
  target = as_.acceptNewGoal()->inter_target;
  
  target_x_ = target.x;
  target_y_ = target.y;
  target_phi_ = target.z;
}

void araControlAction::preemptCB() {
  ROS_INFO("The new goal preemts the old one ");
  // set the action state to be preempted
  as_.setPreempted();
}

void araControlAction::directJoint5manipulate() {
  dynamixel_workbench_msgs::DynamixelState joint5;
  dynamixel_workbench_msgs::DynamixelCommand joint5_ser;
  
  // joint5_ser.request.id=2;
  // joint5_ser.request.value = 15000;
  // joint5_ser.request.addr_name="Goal_Position";
  // client_theta5_.call(joint5_ser);
  // ros::Duration(1.0).sleep();
  
  // send command to rotate the joint 5 - need to change to topics
  joint5.id = 2;  //joint 5
  joint5.name = "Goal_Position";
  joint5.present_position = 0; 
  // ROS_INFO("Inside the direction joint 5 control");
  joint5_pub_.publish(joint5);
}



// } // end of namespace


int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "ara_control");
  

  // ros::NodeHandle nh("~");
  araControlAction  araRobotControl("ara_move");

  ros::Rate loop_rate(10);
   
  // setupserial();
  
  // ros::spinOnce();
  // ros::spin();
  // loop_rate.sleep();
  
  // int mode=0;


  // bool turning_mode, turning;
  // nh.param("turning_mode", turning_mode, true);

  while(ros::ok())
  {
    // araRobotControl.directJoint5manipulate();
    // if (turning_mode)
    // {      
    //   int32_t convPos_joint1=0, convPos_joint3=-50000, convPos_joint4=140000;
    //   int32_t convInter_j1=-150000, convInter_j3=-130000, convInter_j4=272000;
    //   int32_t convInter1_j3=-165000, convInter1_j4=285000;

      // moving to the convenient Pose 2
      // if (convPose_2)
      // {
      //   srv.request.command="";
      //   srv.request.addr_name="Goal_Position";

      //   // moving the joint 6 - keep the safe distance between two feet
      //   srv.request.id=1;
      //   srv.request.value = -7000;
      //   client.call(srv);

      //   // moving the joint 1 the base joint - first
      //   srv.request.id=6;
      //   // srv.request.value = convPos_joint1;
      //   srv.request.value = convPos_joint1;
      //   client.call(srv);
      //   ros::Duration(3.0).sleep();

      //   //moving the joint 3 - the second
      //   srv.request.id=4;
      //   // srv.request.value = convInter1_j3;
      //   // client.call(srv);     

      //   srv.request.id=4;wSgst_f7ubun

      //   srv.request.value = convPos_joint3;
      //   client.call(srv);
      //   ros::Duration(15.0).sleep();

      //   turning = true;
      //   convPose_2 = false;
      //   // ros::Duration(2.0).sleep();
      //   // mobile_config = true;
      // }    

      // make a turn of 180 degree
      // if (turning)
      // {
      //   srv.request.id=5;
      //   int32_t convert1 = 400000;

      //   srv.request.value = convert1; // joint 1    - clockwise - positive
      //   client.call(srv);
      //   ros::Duration(120).sleep(); 

      //   turning = false;
      //   // move_1leg = false;       
      // }

      // change direction
      // if (joint5_turning)
      // {
      //   srv.request.command = "";
        
      //   srv.request.addr_name = "direction";
      //   int8_t id =1;
      //   srv.request.id=id;
      //   //int32_t convert1 = 15000;
      //   ROS_INFO("Running inside the change_orientation");

      //   srv.request.value = 100000; // joint 2 (5)
      //   client.call(srv);

      //   ros::Duration(10).sleep(); 

      //   joint5_turning = false;       
      // }


      // // switching_controller = true;
      // turning_mode = false;
      
    // }

    ros::spinOnce();
    loop_rate.sleep();
// 
  }
  // ros::spinOnce();

  return 0;
}
