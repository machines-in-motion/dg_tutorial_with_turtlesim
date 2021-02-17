/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief
 */

#include "dg_tutorial_with_turtlesim/dynamic_graph_manager/dgm_turtlesim.hpp"

namespace dg_tutorial_with_turtlesim
{
// This is the code of the Dynamic Graph Manager interfacing with turtlesim
// The hardware part inherit from DynamicGraphManager

DgmTurtlesim::DgmTurtlesim() : dynamic_graph_manager::DynamicGraphManager()
{
}

DgmTurtlesim::~DgmTurtlesim()
{
}

// this will be called when creating a new device instance
void DgmTurtlesim::initialize_hardware_communication_process()
{
    std::cout << std::endl
              << "DG TURTLESIM: initializing hard communication ..."
              << std::endl;

    // dynamic graph manager will create nodes. ros_init gives access to them
    dynamic_graph_manager::RosNodePtr node =
        dynamic_graph_manager::get_ros_node("dg_tutorial_with_turtlesim");
    dynamic_graph_manager::ros_add_node_to_executor(
        "dg_tutorial_with_turtlesim");

    // creating the instance of turtlesim (see turtlesim.hpp in same catkin
    // package)
    turtlesim_ = std::make_shared<Turtlesim>(node);

    std::cout << std::endl
              << "DG TURTLESIM: initializing hard communication done."
              << std::endl;
}

// This code is used to fill the "sensors" map with values coming from the robot
// sensors. Note that the keys of the map
// ("position","orientation","odometry_linear_velocity") are the exact same as
// the sensors provided in the configuration file /config/turtlesim.yaml. The
// map will be the "output" of the device, i.e. what can be plugged to the input
// of entities in control graphs
void DgmTurtlesim::get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map)
{
    double x, y, theta, linear_velocity, angular_velocity;

    // reading sensory data from the robot
    // rt_printf("DgmTurtlesim::get_sensors_to_map: Get turtlesim pose.\n");
    turtlesim_->get_pose(x, y, theta);

    // rt_printf("DgmTurtlesim::get_sensors_to_map: Get turtlesim velocity.\n");
    turtlesim_->get_velocity(linear_velocity, angular_velocity);

    assert(x == x);
    assert(y == y);
    assert(linear_velocity == linear_velocity);
    assert(angular_velocity == angular_velocity);

    // filling the sensor maps
    // rt_printf(
    //     "DgmTurtlesim::get_sensors_to_map: Writting in the sensor map.\n");
    map["position"](0) = x;
    map["position"](1) = y;
    map["orientation"](0) = theta;
    map["current_velocity"](0) = linear_velocity;
    map["current_velocity"](1) = angular_velocity;
}

// This code is used to treat commands coming from the graphs. "map" is the
// input of the device, i.e. what can be plugged to output of entities in the
// graph. Note that the keys of the map (linear_velocity, angular_velocity)
// corresponds to controls defined in /config/turtlesim.yaml. Incoming commands
// are fed into turtlesim, having the robot moving.
void DgmTurtlesim::set_motor_controls_from_map(
    const dynamic_graph_manager::VectorDGMap& map)
{
    dynamicgraph::Vector desired_velocity = map.at("desired_velocity");

    turtlesim_->set_velocity(desired_velocity[0], desired_velocity[1]);
}

}  // namespace dg_tutorial_with_turtlesim
