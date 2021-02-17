/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "dynamic_graph_manager/ros.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#define TURTLESIM_POSE_TOPIC "/turtle1/pose"
#define TURTLESIM_CMD_TOPIC "/turtle1/cmd_vel"

namespace dg_tutorial_with_turtlesim
{
class AsyncPose
{
public:
    AsyncPose();
    void get_pose(double &x, double &y, double &theta);
    void get_velocity(double &linear, double &angular);
    void set_pose(double x, double y, double theta);
    void set_velocity(double linear, double angular);

private:
    std::mutex mutex_;
    double x, y, theta;
    double linear_velocity, angular_velocity;
};

class Turtlesim
{
public:
    Turtlesim(dynamic_graph_manager::RosNodePtr node);
    ~Turtlesim();

    void get_pose(double &x, double &y, double &theta);
    void get_velocity(double &linear, double &angular);
    void set_velocity(double linear, double angular);

private:

    std::atomic_bool called_once_;
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);

    AsyncPose pose_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};
}  // namespace dg_tutorial_with_turtlesim
