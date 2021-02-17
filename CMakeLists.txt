#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(dg_tutorial_with_turtlesim)

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Dependencies
#
# depend on ament macros
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS2 dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mim_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
# MiM dependencies.
find_package(mpi_cmake_modules REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(dynamic_graph_manager REQUIRED)
# Dynamic graph dependencies.
find_package(dynamic-graph REQUIRED)
find_package(dynamic-graph-python REQUIRED)

# export de dependencies
ament_export_dependencies(
  ament_cmake
  ament_cmake_python
  std_msgs
  geometry_msgs
  mim_msgs
  rclcpp
  mpi_cmake_modules
  real_time_tools
  dynamic_graph_manager
  dynamic-graph
  dynamic-graph-python)

#
# prepare to export all needed targets
#
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#
# manage python file install
#
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

#
# Install the python ROS nodes
#
install(
  FILES nodes/publish_target_position.py
  RENAME publish_target_position
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
install(
  FILES nodes/sequencer.py
  RENAME sequencer
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)

#
# Library compilation.
#
add_library(${PROJECT_NAME} src/robot_drivers/turtlesim_drivers.cpp
                            src/dynamic_graph_manager/dgm_turtlesim.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
include_directories(${turtlesim_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${turtlesim_LIBRARIES})
target_link_libraries(${PROJECT_NAME}
                      dynamic_graph_manager::dynamic_graph_manager)
target_link_libraries(${PROJECT_NAME} real_time_tools::real_time_tools)
target_link_libraries(${PROJECT_NAME} dynamic-graph::dynamic-graph)
target_link_libraries(${PROJECT_NAME}
                      dynamic-graph-python::dynamic-graph-python)

#
# Install config file
#
set(config_destination "share/${PROJECT_NAME}/dynamic_graph_manager")
install(FILES ${PROJECT_SOURCE_DIR}/config/turtlesim.yaml
        DESTINATION ${config_destination})

#
# Executables
#

# Executable single process
add_executable(dgm_main_turtlesim
               src/dynamic_graph_manager/turtlesim_dg_main.cpp)
target_include_directories(
  dgm_main_turtlesim PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)
target_link_libraries(dgm_main_turtlesim ${PROJECT_NAME})
target_compile_definitions(
  dgm_main_turtlesim
  PUBLIC
    CONFIG_PATH="${CMAKE_INSTALL_PREFIX}/${config_destination}/turtlesim.yaml")

# Executable multi-process
add_executable(dgm_main_turtlesim_multiprocess
               src/dynamic_graph_manager/turtlesim_dg_main_multiprocess.cpp)
target_include_directories(
  dgm_main_turtlesim_multiprocess
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(dgm_main_turtlesim_multiprocess ${PROJECT_NAME})
target_compile_definitions(
  dgm_main_turtlesim_multiprocess
  PUBLIC
    CONFIG_PATH="${CMAKE_INSTALL_PREFIX}/${config_destination}/turtlesim.yaml")

#
# Entity compilation
#
add_library(
  ${PROJECT_NAME}_entities SHARED
  src/dynamic_graph/turtlesim_control_entity.cpp
  src/dynamic_graph/turtlesim_transform_entity.cpp)
target_include_directories(
  ${PROJECT_NAME}_entities
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_entities ${PROJECT_NAME})
# Install the target and it's python bindings.
install_dynamic_graph_plugin_python_bindings(${PROJECT_NAME}_entities)
# Install the plugin.
get_dynamic_graph_plugin_install_path(plugin_install_path)
install(
  TARGETS ${PROJECT_NAME}_entities
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION ${plugin_install_path}
  ARCHIVE DESTINATION ${plugin_install_path}
  RUNTIME DESTINATION ${plugin_install_path}
  INCLUDES
  DESTINATION include)

# Not dynamic graph related: an executable to test ros interface to turtlesim
add_executable(turtlesim_exe src/robot_drivers/turtlesim_exe.cpp)
target_include_directories(
  turtlesim_exe PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
target_link_libraries(turtlesim_exe ${PROJECT_NAME})

#
# Install the package
#

# Install the include directory
install(DIRECTORY include/ DESTINATION include)
# Install the libraries
install(
  TARGETS ${PROJECT_NAME} dgm_main_turtlesim turtlesim_exe
          dgm_main_turtlesim_multiprocess
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

#
# Building documentation.
#
add_documentation()

#
# Export as an ament_package
#
ament_package()
