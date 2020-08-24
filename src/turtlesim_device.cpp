/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck Gesellschaft
 * 
 * @brief 
 */
#include "dg_turtlesim/turtlesim_device.hpp"


namespace dg_turtlesim {

  // This is the code of the Dynamic Graph device
  // interfacing with turtlesim
  

  // The device inherate from DynamicGraphManager
  
  DG_turtlesim_manager::DG_turtlesim_manager()
    : dynamic_graph_manager::DynamicGraphManager() {}

  
  DG_turtlesim_manager::~DG_turtlesim_manager(){}


  // this will be called when creating a new device instance
  void DG_turtlesim_manager::initialize_hardware_communication_process() {

    rt_printf("\nDG TURTLESIM: initializing hard communication ...");

    // dynamic graph manager will create nodes. ros_init gives access to them
    ros::NodeHandle node = dynamic_graph_manager::ros_init("dg_turtlesim");
    
    // creating the instance of turtlesim (see turtlesim.hpp in same catkin package)
    turtlesim_.reset(new Turtlesim( node));
    
    rt_printf(" done\n\n");
    
  }


  // This code is used to fill the "sensors" map with values coming from the robot
  // sensors. Note that the keys of the map ("position","orientation","odometry_linear_velocity")
  // are the exact same as the sensors provided in the configuration file /config/turtlesim.yaml.
  // The map will be the "output" of the device, i.e. what can be plugged to the input of entities in
  // control graphs
  void DG_turtlesim_manager::get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map) {

    double x,y,theta,linear_velocity,angular_velocity;

    // reading sensory data from the robot
    turtlesim_->get_pose( x,
			  y,
			  theta );
    
    turtlesim_->get_velocity( linear_velocity,
			      angular_velocity );

    // filling the sensor maps
    map["position"](0)=x;
    map["position"](1)=y;
    map["orientation"](0)=theta;
    map["current_velocity"](0)=linear_velocity;
    map["current_velocity"](1)=angular_velocity;
    
  }


  // This code is used to treat commands coming from the graphs. "map" is the input of the device,
  // i.e. what can be plugged to output of entities in the graph.
  // Note that the keys of the map (linear_velocity, angular_velocity) corresponds to controls
  // defined in /config/turtlesim.yaml.
  // Incoming commands are fed into turtlesim, having the robot moving.
  void DG_turtlesim_manager::set_motor_controls_from_map(const dynamic_graph_manager::VectorDGMap& map) {

    dynamicgraph::Vector desired_velocity = map.at("desired_velocity");

    turtlesim_->set_velocity( desired_velocity[0],
			      desired_velocity[1] );

  }


}
