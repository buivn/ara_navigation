
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <PinChangeInt.h>
#include <PID_v1.h>

#define pi 3.14159265359
#define motor_pwm 3 // pin with pwm (white wire)
#define motor_dir 4 

#define motor_pwm2 9
#define motor_dir2 12
double kp1 = 0.8 , ki1 = 0.2;             // modify for optimal performance

float energy_l;
float energy_r;

double error_l = 0.0;
double error_r = 0.0;
float del_t = 0.01;
float sum_I_l = 0.0;
float sum_I_r = 0.0;
bool new_Data = false;


int max_pwm = 150;  /*  max speed in rpm  (will measure later) */

ros::NodeHandle nh;

void wheelControl_CallBack(const geometry_msgs::Pose2D &msg) {
  // get the errors from the message
  error_l = msg.x;
  error_r = msg.y;
  new_Data = true;
}

ros::Subscriber<geometry_msgs::Pose2D> s("/wheels_move", &wheelControl_CallBack);

void setup() 
{
  pinMode(motor_pwm, OUTPUT);
  pinMode(motor_dir, OUTPUT);
  pinMode(motor_pwm2, OUTPUT);
  pinMode(motor_dir2, OUTPUT);
  
  nh.initNode();
  nh.subscribe(s);
}

void loop() 
{
  if (new_Data) {
    sum_I_l += error_l*del_t;
    sum_I_r += error_r*del_t;
    energy_l = kp1*error_l + ki1*sum_I_l;
    energy_r = kp1*error_r + ki1*sum_I_r;
    
    /*  scale  & set it to positive if negative*/ 
    int l_pwm = map(abs(energy_l), 0, max_pwm, 0 , 255); 
    int r_pwm = map(abs(energy_r), 0, max_pwm, 0 , 255); 
  
    /*  in case input speed is above max speed  */
    if (l_pwm > 250)   l_pwm = 200; 
     // nh.logwarn("Warnings: left speed is above limit");
    if (r_pwm > 250)   r_pwm = 200; 
  
    /*  left wheel move  */
    if (energy_l > 0) leftForward(l_pwm);
    if (energy_l < 0) leftBackwards(abs(l_pwm));
  
    /*  right wheel move  */
    if (energy_r > 0)  rightForward(r_pwm);
    if (energy_r < 0)  rightBackwards(abs(r_pwm));
    new_Data = false;
  }
  nh.spinOnce();
  delay(1);
} 


void leftForward(int pwm)
{
  analogWrite(motor_pwm, pwm); 
//  analogWrite(motor_pwm, 0); /
  digitalWrite(motor_dir, 1);
}

void leftBackwards(int pwm)
{
  analogWrite(motor_pwm, pwm); 
  digitalWrite(motor_dir, 0);
}

void rightForward(int pwm)
{
  analogWrite(motor_pwm2, pwm); 
//  analogWrite(motor_pwm2, 0); /
  digitalWrite(motor_dir2, 1);
}

void rightBackwards(int pwm)
{
  analogWrite(motor_pwm2, pwm); 
  digitalWrite(motor_dir2, 0);
}
