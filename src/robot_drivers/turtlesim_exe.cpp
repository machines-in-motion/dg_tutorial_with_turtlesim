#include <unistd.h>

#include <cstdlib>
#include <iostream>

#include "dg_tutorial_with_turtlesim/robot_drivers/turtlesim_drivers.hpp"

bool parse_arguments(int argc, char *argv[], double &linear, double &angular)
{
    if (argc != 3)
    {
        std::cout
            << "\n\nusage: turtlesim_exe linear_velocity angular_velocity\n\n";
        return false;
    }

    linear = static_cast<double>(atof(argv[1]));
    angular = static_cast<double>(atof(argv[2]));

    return true;
}

void display_pose(dg_tutorial_with_turtlesim::Turtlesim &turtle)
{
    double pose_x, pose_y, pose_theta;
    turtle.get_pose(pose_x, pose_y, pose_theta);
    std::cout << "\nTurtlesim:";
    std::cout << "\tx: " << pose_x;
    std::cout << "\ty: " << pose_y;
    std::cout << "\ttheta: " << pose_theta << "\n";
}

int main(int argc, char *argv[])
{
    double linear, angular;
    bool ok = parse_arguments(argc, argv, linear, angular);

    if (!ok)
    {
        return 0;
    }

    dynamic_graph_manager::RosNodePtr ros_node =
        dynamic_graph_manager::get_ros_node("turtlesim_interface");
    dynamic_graph_manager::ros_add_node_to_executor("turtlesim_interface");
    dynamic_graph_manager::ros_spin_non_blocking();

    dg_tutorial_with_turtlesim::Turtlesim turtle(ros_node);
    display_pose(turtle);
    long total_time_waited_us = 0;
    std::cout << "\nApplying linear: " << linear;
    std::cout << " angular: " << angular << " for 5 seconds\n";

    // running for 5 seconds
    while (total_time_waited_us <= 5000000)
    {
        turtle.set_velocity(linear, angular);
        usleep(5000);
        total_time_waited_us += 5000;
        display_pose(turtle);
    }

    dynamic_graph_manager::ros_shutdown();
}
