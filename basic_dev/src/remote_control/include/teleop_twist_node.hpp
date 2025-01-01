#ifndef TELEOP_TWIST_NODE
#define TELEOP_TWIST_NODE

#include <ros/ros.h>
#include "ros/node_handle.h"
#include "ros/timer.h"
#include "airsim_ros/VelCmd.h"

#define KEYCODE_RIGHT 0x43 
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71

class TeleopRmua
{
public:
  TeleopRmua(ros::NodeHandle &nh);
  void keyLoop();

private:
  void TimedCallback(const ros::TimerEvent& e);

  bool dirty_ = false;
  airsim_ros::VelCmd velcmd_;
  ros::NodeHandle nh_;
  double l_scale_x_, l_scale_y_, l_scale_z_, a_scale_z_;
	ros::Publisher vel_publisher;  
  ros::Timer timer_;
};



#endif