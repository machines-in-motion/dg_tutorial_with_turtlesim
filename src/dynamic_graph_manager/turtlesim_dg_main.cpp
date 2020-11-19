
#include "dg_tutorial_with_turtlesim/dynamic_graph_manager/dgm_turtlesim.hpp"


int main(int /*argc*/, char* /*argv*/[]) {
  
  std::string yaml_params_file = std::string(CONFIG_PATH);

  std::cout << "Loading parameters from "
            << yaml_params_file
            << std::endl;
  YAML::Node param = YAML::LoadFile(yaml_params_file);

  dg_tutorial_with_turtlesim::DgmTurtlesim dgm;

  dgm.initialize(param);
  dgm.run_single_process(); 
}
