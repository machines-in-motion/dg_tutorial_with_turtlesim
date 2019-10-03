cmake_minimum_required(VERSION 2.8)
project(dg_turtlesim)

#set(ROBOT iiwa)

#INCLUDE(cmake/base.cmake)

#find_package(catkin REQUIRED)
#find_package(Eigen3 REQUIRED)

# support for c++ 11
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

# Search for dependencies
set(CATKIN_PKGS ${CATKIN_PKGS}
  mpi_cmake_modules
  real_time_tools
  dynamic_graph_manager
  std_msgs
  geometry_msgs
  message_generation
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})
search_for_python()
search_for_boost()
search_for_eigen()

add_service_files(
  FILES
  turtlesim_target.srv )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Catkin export
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dg_turtlesim
  EXECUTABLES dg_turtlesim_run
  CATKIN_DEPENDS ${CATKIN_PKGS}
)


# Trigger dependency to dynamic-graph.
add_required_dependency("dynamic-graph >= 3.0.0")
add_required_dependency("dynamic-graph-python >= 3.0.0")

#ADD_COMPILE_DEPENDENCY("pinocchio >= 1.3.0")

#build_prefix_for_pkg("dynamic-graph" dg_name)
#build_prefix_for_pkg("dynamic-graph-python" dgp_name)

#link_directories(
#  ${${dg_name}_LIBRARY_DIRS}
#  ${${dgp_name}_LIBRARY_DIRS}
#)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_PATH}
    ${Boost_INCLUDE_DIR}
    ${Eigen_INCLUDE_DIRS}
)

# Library compilation
add_library( dg_turtlesim
  src/turtlesim.cpp
  src/turtlesim_device.cpp )
target_link_libraries( dg_turtlesim ${catkin_LIBRARIES})
pkg_config_use_dependency(dg_turtlesim dynamic-graph)
pkg_config_use_dependency(dg_turtlesim dynamic-graph-python)

# Executable compilation
add_executable ( dg_turtlesim_run
  src/turtlesim_dg_main.cpp )
target_link_libraries( dg_turtlesim_run dg_turtlesim)
target_compile_definitions(dg_turtlesim_run PUBLIC
  DG_TURTLESIM_CONFIG_PATH="${PROJECT_SOURCE_DIR}/config/turtlesim.yaml"
  )

# Entity compilation
add_library( dg_turtlesim_entities SHARED
  src/turtlesim_control_entity.cpp
  src/turtlesim_transform_entity.cpp)
target_link_libraries(dg_turtlesim_entities dg_turtlesim)

# Export entities as dg plugins, including python wrapper
dynamic_graph_python_module("dg_turtlesim_entities"
  dg_turtlesim_entities
  dg_turtlesim_entities_wrap )
set_target_properties(dg_turtlesim_entities PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${DYNAMIC_GRAPH_PLUGIN_DIR} )

# Not dynamic graph related: an executable to test ros interface
# to turtlesim
add_executable(turtlesim_exe
  src/turtlesim.cpp
  src/turtlesim_exe.cpp)
target_link_libraries(turtlesim_exe ${catkin_LIBRARIES})


