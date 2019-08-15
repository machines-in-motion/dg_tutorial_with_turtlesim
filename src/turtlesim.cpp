#include "dg_turtlesim/turtlesim.hpp"


namespace dg_turtlesim {

  class AsyncPose {

  public:

    void get_pose(double &x, double &y, double &theta){
      this->mutex_.lock();
      x = this->x;
      y = this->y;
      theta = this->theta;
      this->mutex_.unlock();
    }
    
    void get_velocity(double &linear, double &angular){
      this->mutex_.lock();
      linear = this->linear_velocity;
      angular = this->angular_velocity;
      this->mutex_.unlock();
    }

    void set_pose(double x, double y, double theta){
      this->mutex_.lock();
      this->x = x;
      this->y = y;
      this->theta = theta;
      this->mutex_.unlock();
    }
    
    void set_velocity(double linear, double angular){
      this->mutex_.lock();
      this->linear_velocity = linear;
      this->angular_velocity = angular;
      this->mutex_.unlock();
    }
    
  private:
    std::mutex mutex_;
    double x,y,theta;
    double linear_velocity, angular_velocity;
    
  };


  static AsyncPose POSE;
  

  void pose_callback(const turtlesim::Pose::ConstPtr& data){
    
    POSE.set_pose(data->x,
		  data->y,
		  data->theta);

    POSE.set_velocity(data->linear_velocity,
		      data->angular_velocity);
    
  }
  
  
  Turtlesim::Turtlesim(ros::NodeHandle &node){

    subscriber_ = node.subscribe( TURTLESIM_POSE_TOPIC,
				 5,
				 pose_callback );

    publisher_ = node.advertise<geometry_msgs::Twist>(TURTLESIM_CMD_TOPIC,
						     100);

  }

  
  Turtlesim::~Turtlesim(){}

  
  
  void Turtlesim::get_pose(double &x, double &y, double &theta){

    POSE.get_pose(x,y,theta);
    
  }


  void Turtlesim::get_velocity(double &linear, double &angular){

    POSE.get_velocity(linear,angular);
    
  }

  
  void Turtlesim::set_velocity(double linear, double angular){

    geometry_msgs::Twist twist;
    
    twist.linear.x = linear;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angular;
    
    publisher_.publish(twist);
    
  }


  
  
  

}
