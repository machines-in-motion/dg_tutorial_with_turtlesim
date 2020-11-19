/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck Gesellschaft
 * 
 * @brief Expose the vicon_client_entity dynamic_graph module to python.
 */

#include "dg_tutorial_with_turtlesim/dynamic_graph/turtlesim_control_entity.hpp"
#include "dg_tutorial_with_turtlesim/dynamic_graph/turtlesim_transform_entity.hpp"

typedef boost::mpl::vector< 
    dg_tutorial_with_turtlesim::TurtlesimControl,
    dg_tutorial_with_turtlesim::TurtlesimTransform
> entities_t;