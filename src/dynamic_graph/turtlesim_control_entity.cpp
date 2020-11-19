#include "dg_tutorial_with_turtlesim/dynamic_graph/turtlesim_control_entity.hpp"

#include <iostream>

namespace dg_tutorial_with_turtlesim
{
// Register new Entity type in the factory
// Note that the second argument is the type name of the python class
// that will be created when importing the python module.
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TurtlesimControl, "TurtlesimControl");

const double TurtlesimControl::TIME_STEP_DEFAULT = .005;

const std::string& TurtlesimControl::getClassName() const
{
    return CLASS_NAME;
}

std::string TurtlesimControl::getDocString() const
{
    return " proportional control for velocity of the turtlesim";
}

TurtlesimControl::TurtlesimControl(const std::string& inName)

    : dynamicgraph::Entity(inName),

      errorSIN(NULL, "TurtlesimControl(" + inName + ")::input(vector)::error"),

      velocitySOUT(
          boost::bind(&TurtlesimControl::control, this, _1, _2),
          errorSIN,
          "TurtlesimControl(" + inName + ")::output(vector)::desired_velocity")
{
    signalRegistration(errorSIN);
    signalRegistration(velocitySOUT);

    std::string docstring;
    docstring = "\n Set proportional gain (double) \n";
    addCommand(std::string("setKp"),
               new ::dynamicgraph::command::Setter<TurtlesimControl,
                                                   dynamicgraph::Vector>(
                   *this, &TurtlesimControl::setKp, docstring));
}

void TurtlesimControl::setKp(const dynamicgraph::Vector& proportional_gains)
{
    if (proportional_gains_.size() == 0)
    {
        proportional_gains_.resize(proportional_gains.size());
    }
    for (int i = 0; i < proportional_gains.size(); i++)
    {
        proportional_gains_[i] = proportional_gains[i];
    }
}

dynamicgraph::Vector& TurtlesimControl::control(
    dynamicgraph::Vector& target_velocity, const int& inTime)
{
    const dynamicgraph::Vector& error = errorSIN(inTime);

    if (target_velocity.size() != 2)
    {
        target_velocity.resize(2);
    }

    // linear velocity:
    double linear;
    // not going forward if target in the back
    if (error[0] < 0)
    {
        linear = 0;
    }
    // P controller if target in front
    else
    {
        linear = proportional_gains_[0] * error[0];
    }

    // angular velocity, P controller
    double angular;
    double angle = atan2(error[1], error[0]);
    angular = proportional_gains_[1] * angle;

    target_velocity[0] = linear;
    target_velocity[1] = angular;

    return target_velocity;
}

void TurtlesimControl::display(std::ostream& os) const
{
    os << "TurtlesimControl " << getName();

    try
    {
        os << "control = " << velocitySOUT;
    }
    catch (dynamicgraph::ExceptionSignal e)
    {
    }
}

}  // namespace dg_tutorial_with_turtlesim
