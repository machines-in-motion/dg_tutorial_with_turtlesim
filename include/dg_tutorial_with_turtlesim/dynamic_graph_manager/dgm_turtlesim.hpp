/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck Gesellschaft
 * 
 * @brief 
 */

// real-time prints
#include "real_time_tools/iostream.hpp"
// dynamic graph manager
#include "dynamic_graph_manager/dynamic_graph_manager.hpp"
// robot drivers
#include "dg_tutorial_with_turtlesim/robot_drivers/turtlesim_drivers.hpp"


namespace dg_tutorial_with_turtlesim {
    
  
class DgmTurtlesim : public dynamic_graph_manager::DynamicGraphManager {

public:


  typedef std::shared_ptr<Turtlesim> Turtlesim_ptr;

  
  DgmTurtlesim();
  ~DgmTurtlesim();
  
  void initialize_hardware_communication_process();
  void get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map);
  void set_motor_controls_from_map(const dynamic_graph_manager::VectorDGMap& map);

  
private:

  // hides the ros machinery for communicating with the turtlesim
  Turtlesim_ptr turtlesim_;

};

  
} 
