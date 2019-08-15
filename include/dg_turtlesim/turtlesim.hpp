#pragma once


#include "ros/spinner.h"
#include "ros/ros.h"
#include "ros/master.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <stdexcept>
#include <mutex>
#include <memory>

#define TURTLESIM_POSE_TOPIC "/turtle1/pose"
#define TURTLESIM_CMD_TOPIC "/turtle1/cmd_vel"

namespace dg_turtlesim {


  class Turtlesim {

  public:

    Turtlesim(ros::NodeHandle &node);
    ~Turtlesim();
    
    void get_pose(double &x, double &y, double &theta);
    void get_velocity(double &linear, double &angular);
    void set_velocity(double linear, double angular);
    

  private:

    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    
  };



}
