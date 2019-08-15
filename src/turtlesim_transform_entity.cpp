#include <iostream>
#include "dg_turtlesim/turtlesim_transform_entity.hpp"


namespace dg_turtlesim {


  // Register new Entity type in the factory
  // Note that the second argument is the type name of the python class
  // that will be created when importing the python module.
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TurtlesimTransform, "TurtlesimTransform");

  
  const double TurtlesimTransform::TIME_STEP_DEFAULT = .005;

  
  const std::string& TurtlesimTransform::getClassName() const {
    return CLASS_NAME;
  }

  
  std::string TurtlesimTransform::getDocString() const {
    return "transform from absolute position to position relative to turtlesim";
  }
  
  
  TurtlesimTransform::TurtlesimTransform(const std::string  &inName)

    // an entity must inherte from dynamicgraph Entity
    : dynamicgraph::Entity(inName),

      // we define here the input signals, i.e. data that will be required to compute
      // the output signals 

      // the second argument (string) will be useful to provide information to the users
      // composing graphs using python. The last substring (after '::')
      // will be the attribute name of the signal in Python.

      // the first argument (NULL) indicates the signal does not depends on another
      // signal for computation. This will be always NULL for input signals.
      
      robot_absolute_positionSIN(NULL,
      				 "TurtleSimTransform("+name+")::input(vector2d)::robot_absolute_position"),
      robot_orientationSIN(NULL,
      			   "TurtlesimTransform("+name+")::input(double)::robot_orientation"),
      absolute_positionSIN(NULL,
			   "TurtlesimTransform("+name+")::input(vector2d)::absolute_position"),

      // we define here the output signal of the entity. 

      // The first argument indicates that all three input signals are required for computation of
      // this output signal.
      
      relative_positionSOUT(robot_absolute_positionSIN << robot_orientationSIN << absolute_positionSIN,
			    "TurtlesimTransform("+name+")::output(vector2d)::relative_position")

      
  {

    // registering the signals
    
    signalRegistration(robot_absolute_positionSIN);
    signalRegistration(robot_orientationSIN);
    signalRegistration(absolute_positionSIN);
    signalRegistration(relative_positionSOUT);

    // indicating which function will be used for computing the output signal.
    // (see code for transform right below)
    relative_positionSOUT.setFunction( boost::bind( &TurtlesimTransform::transform,
						   this,
						   _1,_2) );
    
  }


  // Function for computing the output signal.
  // Note that the output signal is both the input argument ("dynamicgraph::Vector& relative_position")
  // and the returned value (dynamicgraph::Vector&) of the function.

  dynamicgraph::Vector& TurtlesimTransform::transform(dynamicgraph::Vector& relative_position,
						      const int &inTime){


    if (relative_position.size()!=2){
      relative_position.resize(2);
    }

    // getting the values of input signals
    
    const dynamicgraph::Vector& robot_absolute_position = robot_absolute_positionSIN.access(inTime);
    double theta = robot_orientationSIN.access(inTime)[0];
    const dynamicgraph::Vector& absolute_position = absolute_positionSIN.access(inTime);

    // computing the output signal (position in the robot frame)
    // and returning it
    
    for(int i=0;i<2;i++){
      diff_position_[i] = absolute_position[i]-robot_absolute_position[i];
    }

    double gamma = atan2(diff_position_[1],diff_position_[0]);
    double beta = gamma-theta;
    
    double s = 0;
    for(int i=0;i<2;i++){
      s+= diff_position_[i]*diff_position_[i];
    }
    double d = sqrt(s);

    
    relative_position[0]=d*cos(beta);
    relative_position[1]=d*sin(beta);

    return relative_position;
    
  }


  // to get documentation when composing graph using python
  
  void TurtlesimTransform::display( std::ostream& os ) const {

    os << "TurtlesimTransform "<< getName();
    
    try {
      
      os <<"relative position = "<< relative_positionSOUT;
      
    } catch (dynamicgraph::ExceptionSignal e) {}
    
  }
  
  

  
}
