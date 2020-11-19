/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck Gesellschaft
 * 
 * @brief 
 */

#pragma once

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/factory.h>
#include <math.h>
#include <string>


namespace dg_tutorial_with_turtlesim {

  class TurtlesimTransform : public dynamicgraph::Entity {


  public:

    TurtlesimTransform( const std::string & name );

    static const std::string CLASS_NAME;
    static const double TIME_STEP_DEFAULT;   
    virtual std::string getDocString () const;
    virtual const std::string& getClassName( void ) const;
    virtual void display( std::ostream& os ) const;
    
    dynamicgraph::Vector& transform( dynamicgraph::Vector& relative_position,
				     const int& time );
    
  private:

    double diff_position_[2];
    
    // positions are [x,y]
    // absolute 2d position of the robot. Would fit to plug to device sensors position
    dynamicgraph::SignalPtr<dynamicgraph::Vector,int> robot_absolute_positionSIN;
    
    // orientation of the robot. Would fit to plug device sensors orientation
    dynamicgraph::SignalPtr<dynamicgraph::Vector,int> robot_orientationSIN;
    
    // absolute 2d position of an arbitrary object in the turtlesim 2d world
    dynamicgraph::SignalPtr<dynamicgraph::Vector,int> absolute_positionSIN;
    
    // 2d position of this arbitrary object transformed in the turtlesim 2d frame
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector,int> relative_positionSOUT;

};

}
