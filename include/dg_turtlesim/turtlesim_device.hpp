
#include "real_time_tools/iostream.hpp"

#include "ros/ros.h"
#include "ros/master.h"
#include "dynamic_graph_manager/ros_init.hh"
#include "dynamic_graph_manager/dynamic_graph_manager.hh"

#include "dg_turtlesim/turtlesim.hpp"


namespace dg_turtlesim {
    
  
class DG_turtlesim_manager : public dynamic_graph::DynamicGraphManager {

public:


  typedef std::shared_ptr<Turtlesim> Turtlesim_ptr;

  
  DG_turtlesim_manager();
  ~DG_turtlesim_manager();
  
  void initialize_hardware_communication_process();
  void get_sensors_to_map(dynamic_graph::VectorDGMap& map);
  void set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map);

  
private:

  // hides the ros machinery for communicating with the turtlesim
  Turtlesim_ptr turtlesim_;

};

  
} 
