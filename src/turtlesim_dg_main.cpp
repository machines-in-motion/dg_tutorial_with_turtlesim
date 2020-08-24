
#include "dg_turtlesim/turtlesim_device.hpp"


int main(int argc, char*argv[]) {
  
  std::string yaml_params_file = std::string(CONFIG_PATH);

  std::cout << "Loading parameters from "
            << yaml_params_file
            << std::endl;
  YAML::Node param = YAML::LoadFile(yaml_params_file);

  dg_turtlesim::DG_turtlesim_manager dgm;

  dgm.initialize(param);
  dgm.run();
  ros::waitForShutdown();
  
}
