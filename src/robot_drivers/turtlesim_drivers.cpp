#include "dg_tutorial_with_turtlesim/robot_drivers/turtlesim_drivers.hpp"

#include "real_time_tools/timer.hpp"

namespace dg_tutorial_with_turtlesim
{
/*
 * AsyncPose
 */
void AsyncPose::get_pose(double &x, double &y, double &theta)
{
    this->mutex_.lock();
    x = this->x;
    y = this->y;
    theta = this->theta;
    this->mutex_.unlock();
}

void AsyncPose::get_velocity(double &linear, double &angular)
{
    this->mutex_.lock();
    linear = this->linear_velocity;
    angular = this->angular_velocity;
    this->mutex_.unlock();
}

void AsyncPose::set_pose(double x, double y, double theta)
{
    this->mutex_.lock();
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->mutex_.unlock();
}

void AsyncPose::set_velocity(double linear, double angular)
{
    this->mutex_.lock();
    this->linear_velocity = linear;
    this->angular_velocity = angular;
    this->mutex_.unlock();
}

/*
 * Turtlesim
 */
Turtlesim::Turtlesim(dynamic_graph_manager::RosNodePtr node)
{
    // Convenient shortcut.
    using std::placeholders::_1;
    // Create the subscrition.
    subscription_ = node->create_subscription<turtlesim::msg::Pose>(
        TURTLESIM_POSE_TOPIC,
        5,
        std::bind(&Turtlesim::pose_callback, this, _1));

    publisher_ = node->create_publisher<geometry_msgs::msg::Twist>(
        TURTLESIM_CMD_TOPIC, 100);

    called_once_ = false;
}

Turtlesim::~Turtlesim()
{
}

void Turtlesim::get_pose(double &x, double &y, double &theta)
{
    while (!called_once_)
    {
        real_time_tools::Timer::sleep_sec(0.1);
    }
    pose_.get_pose(x, y, theta);
}

void Turtlesim::get_velocity(double &linear, double &angular)
{
    while (!called_once_)
    {
        real_time_tools::Timer::sleep_sec(0.1);
    }
    pose_.get_velocity(linear, angular);
}

void Turtlesim::set_velocity(double linear, double angular)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angular;
    publisher_->publish(twist);
}

void Turtlesim::pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
    called_once_ = true;
    pose_.set_pose(msg.get()->x, msg.get()->y, msg.get()->theta);
    pose_.set_velocity(msg.get()->linear_velocity, msg.get()->angular_velocity);
}

}  // namespace dg_tutorial_with_turtlesim
