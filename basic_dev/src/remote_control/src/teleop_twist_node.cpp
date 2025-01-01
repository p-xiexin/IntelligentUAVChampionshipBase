#include "teleop_twist_node.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_rmua");
  ros::NodeHandle nh;
  TeleopRmua teleop_rmua(nh);

  signal(SIGINT,quit);
  ros::AsyncSpinner spinner(1); // 使用一个线程来处理回调
  spinner.start();
  teleop_rmua.keyLoop();
  spinner.stop();
  
  return(0);
}

TeleopRmua::TeleopRmua(ros::NodeHandle &nh):
  nh_(nh),
  l_scale_x_(2.0),
  l_scale_y_(2.0),
  l_scale_z_(2.0),
  a_scale_z_(2.0)
{
  vel_publisher = nh_.advertise<airsim_ros::VelCmd>("airsim_node/drone_1/vel_cmd_body_frame", 1);
  timer_ = nh_.createTimer(ros::Duration(0.05), &TeleopRmua::TimedCallback, this, false, true);
}

void TeleopRmua::TimedCallback(const ros::TimerEvent& e) {
  if(dirty_ == true)
  {
    vel_publisher.publish(velcmd_);
    dirty_ = false;
  }
  else
  {
    velcmd_.twist.angular.z = 0;
    velcmd_.twist.linear.x = 0;
    velcmd_.twist.linear.y = 0;
    velcmd_.twist.linear.z = 0;
    vel_publisher.publish(velcmd_);
  }
}

void TeleopRmua::keyLoop()
{
  char c;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the uav.");


  while(ros::ok())
  {
    // get the next event from the keyboard  
    if(::read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    double linear_x = 0, linear_y = 0, linear_z = 0, angular_z = 0;
    // ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_LEFT:
        // std::cout << "LEFT" << std::endl;
        linear_y = 1.0;
        dirty_ = true;
        break;
      case KEYCODE_RIGHT:
        // std::cout << "RIGHT" << std::endl;
        linear_y = -1.0;
        dirty_ = true;
        break;
      case KEYCODE_UP:
        // std::cout << "UP" << std::endl;
        linear_x = 1.0;
        dirty_ = true;
        break;
      case KEYCODE_DOWN:
        // std::cout << "DOWN" << std::endl;
        linear_x = -1.0;
        dirty_ = true;
        break;
      case KEYCODE_W:
        // std::cout << "RISE" << std::endl;
        linear_z = -1.0;
        dirty_ = true;
        break;
      case KEYCODE_S:
        // std::cout << "FALL" << std::endl;
        linear_z = 1.0;
        dirty_ = true;
        break;
      case KEYCODE_A:
        // std::cout << "YAW" << std::endl;
        angular_z = -1.0;
        dirty_ = true;
        break;
      case KEYCODE_D:
        // std::cout << "YAW" << std::endl;
        angular_z = 1.0;
        dirty_ = true;
        break;
      case KEYCODE_Q:
        break;
    }
   
    velcmd_.twist.angular.z = a_scale_z_*angular_z;
    velcmd_.twist.linear.x = l_scale_x_*linear_x;
    velcmd_.twist.linear.y = l_scale_y_*linear_y;
    velcmd_.twist.linear.z = l_scale_z_*linear_z;
  }

  return;
}