#pragma once

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>



namespace dg_turtlesim {

  class TurtlesimControl : public dynamicgraph::Entity {


  public:

    TurtlesimControl( const std::string & name );

    static const std::string CLASS_NAME;
    static const double TIME_STEP_DEFAULT;   
    virtual std::string getDocString () const;
    virtual const std::string& getClassName( void ) const;
    virtual void display( std::ostream& os ) const;
    
    dynamicgraph::Vector& control(dynamicgraph::Vector& target_velocity, const int& time);
    
    void setKp(const dynamicgraph::Vector &proportional_gains);

  private:

    dynamicgraph::Vector proportional_gains_;
    dynamicgraph::SignalPtr<dynamicgraph::Vector,int> errorSIN;
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector,int> velocitySOUT;
    
};

}
