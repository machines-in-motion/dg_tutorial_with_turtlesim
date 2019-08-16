# Dynamic graph tutorial 

## What this is

Dynamic Graph Manager is a software for composing realtime control graph for robotics.

This tutorial is a rapid way to grasp of what dynamic graph is and how it works. It also presents a concrete exemple of its usage. In this tutorial, we will show how to control a [turtlesim](http://wiki.ros.org/turtlesim) via dynamic graph.

## Intuition 

Dynamic graph allows to compose control graphs for your favorite robot. To get your robot to work with Dynamic Graph, you need to wrap your robot code into a **device**. A device has ouputs (robot sensors) and inputs (control commands). You can compose control graphs that connect the device ouputs to the device inputs. Composing a graph is done by associating entities one to another by plugging outputs to inputs. 

In Dynamic graph, a device/entity output/input is called a **signal**. An entity has input signals and output signals. 

You may find the concept similar to ROS. In ROS, you compose graphs by associating nodes together, association of nodes being done via services / subscriptions / publications. Here the fundamental difference:

- In ROS, nodes runs asynchronuously (i.e. each node may run its own process). 
- In Dynamic Graph, the graph is fully walked once per iteration. All entities are hosted by the same process and run synchronuously. If running on a realtime OS (rt-preempt patch is supported), the execution of the graph will be realtime safe.

Dynamic graph entities and devices must be programmed in realtime friendly C++ code. But composition of the graph can be done in python. Once you have a large collection of entities, programming in C++ may become rare.

It is possible to interact asynchronuously with entities of the graph during runtime via python commands. For example, if a control entity supports it, you may change the gains it uses during runtime by calling a python function.

Some robot independant entities are already there to be used. For example, the rosSubscribe entity allows to "input" data from ROS to a graph during runtime.

This tutorial will provides examples of devices, entities and graphs.


## Preparing your machine

- Install ubuntu 16.04

- Clone "ubuntu installation script"

```bash
cd /tmp
git clone https://github.com/machines-in-motion/ubuntu_installation_scripts.git
```
- Run the script (will take a while)

```bash
sudo ./ubuntu_installation_scripts/official/setup_ubuntu install
```

---

*What this script does*

The script will install various dependencies via "apt-get install" and pip. These dependencies include ROS and Dynamic Graph. To see the complete list of operations performed, visit the Dockerfile in the folder: 

/ubuntu_installation_scripts/official/ubuntu_16_04/docker

The script does not do anything fancy (e.g. messing up with farious configuration files), and should not break anything that you already have running on your desktop.

---

## Preparing and compiling your workspace


- Create a directory for your workspace (we will use ~/Workspace in this tutorial, but can be anything) 

```bash
cd ~
mkdir Workspace
cd Workspace
```

- Clone treep configuration folder "machines in motion"

```bash
cd ~/Workspace 
git clone https://github.com/machines-in-motion/treep_machines_in_motion.git
```
---

treep is a project manager, i.e. an executable that helps managing several repositories grouped into projects (e.g. cloning all the repos at once). treep_machines_in_motion is a configuration folder for treep, i.e. where repositories and projects are defined. See [treep doc](https://pypi.org/project/treep/)

---

- Use treep to clone all required repositories. The command below will clone all the repositories containing the code used for this tutorial. Note that your ssh-key for gihub needs to be active (see [documentation](https://help.github.com/en/enterprise/2.16/user/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent))

```bash
cd ~/Workspace
treep --clone DG_TUTORIAL
treep --status
```

---

The commands above download the source of dynamic graph, in Workspace/src/not_catkin. 
But they will be unused in this tutorials. Only the binaries, that have been installed in /opt/openrobots/ (when running the script "setup_ubuntu install" above),  will be. 

---

- Compiling the code using catkin

```bash
cd ~/Workspace/workspace
source /opt/ros/kinetic/setup.bash # activating ROS and catkin
source /opt/openrobots/setup.bash # activating dynamic graph
catkin_make install
```

---

[catkin](http://wiki.ros.org/catkin/conceptual_overview) is a tool working on top of cmake. This is the compilation manager used by ROS. All the code used in this tutorial is organized in catkin packages you will find in the subfolders of ~/Workspace/workspace/src.

---

### Running the unit tests (optional)

```bash
cd ~/Workspace/workspace
source ./devel/setup.bash
catkin_make run_tests
```

### Compiling and accessing the documentation (optional)

```bash
cd ~/Workspace/workspace
source ./devel/setup.bash
catkin_make -DBUILD_DOCUMENTATION=ON
```

You will find index.html in ~/Workspace/workspace/devel/share/dynamic_graph_manager/doc/html

### Sourcing automatically your workspace

This tutorial will assume that in each terminal you open, dynamic graph and your compiled workspace are sourced:

```bash
source /opt/openrobots/setup.bash
cd ~/Workspace/workspace
source ./devel/setup.bash
```

To avoid having to type this in each new terminal you open, add these lines in ~/.bashrc :

```bash
echo "Sourcing DG TUTORIAL"
source /opt/openrobots/setup.bash
source ~/Workspace/workspace/devel/setup.bash
```

## Creating a device (C++)

A device is a wrapper over the sensors and actuators of your robot to make it compatible with Dynamic graph. In this section, we will create a device for a turtlesim robot.

### What is turtlesim

Turtlesim is a cute simulated robot. It is famous for being used in ROS tutorials. See : [http://wiki.ros.org/turtlesim](http://wiki.ros.org/turtlesim).

You can start it:

- in a first terminal, start a roscore

```bash
roscore
```

- In a *second* terminal, start a turtlesim node:

```bash
rosrun turtlesim turtlesim_node
```

A window hosting a little turtle should appear.

![turtlesim](http://wiki.ros.org/turtlesim?action=AttachFile&do=get&target=turtlesim.png)

A turtlesim is control by sending velocity command to it via a ROS service. For example, you can send a "velocity impulse" by publishing a command (from yet another terminal) :

```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0" 
```
You may get sensory information by subscribing to the right topic:

```bash
rostopic echo /turtle1/pose
``` 

### Dynamic graph device over turtlesim

Our dynamic graph device will wrap up the turtlesim to make it compatible with dynamic graph. The output of the device (i.e. the sensory information) will be generated from the pose published by the turtlesim. The input to the device (i.e. velocity commands) will be used to publish velocity commands.

#### Generic wrapper over turtlesim

We created a generic c++ turtlesim object ("generic" here means dynamic-graph independant) which allows to interact programmatically with the turtlesim, hidding ROS commands under the hood.  

This is generic ROS / C++ programming, so this code is out of the scope of this tutorial. The final results is this API:

```cpp

namespace dg_turtlesim {

  class Turtlesim {

  public:

    Turtlesim(ros::NodeHandle &node);
    ~Turtlesim();
    
    // under the hood: read data from ros subscription
    void get_pose(double &x, double &y, double &theta);
    void get_velocity(double &linear, double &angular);

    // under the hood: publish velocity commands
    void set_velocity(double linear, double angular);
    
  };

}

```

If you are curious, you can see the source code

```bash

# 'roscd + <catkin package name>' can be used to 'cd' to the source of the package
# the source file will be turtlesim.cpp
roscd dg_turtlesim/src

```

### Turtlesim dynamic graph device

The turtlesim device will define for the control graphs the output signals from the robot (sensory information) and the input signals to the robot (velocity command).

The code for the device is in dg_turtlesim/src/turtlesim_device.cpp

```cpp
#include "dg_turtlesim/turtlesim_dynamic_graph_manager.hpp"


namespace dg_turtlesim {

  // This is the code of the Dynamic Graph device
  // interfacing with turtlesim
  

  // The device must inherate from DynamicGraphManager
  DG_turtlesim_manager::DG_turtlesim_manager()
    : dynamic_graph::DynamicGraphManager() {}

  
  DG_turtlesim_manager::~DG_turtlesim_manager(){}


  // this will be called when creating a new device instance
  void DG_turtlesim_manager::initialize_hardware_communication_process() {

    rt_printf("\nDG TURTLESIM: initializing hard communication ...");

    // dynamic graph manager automatically creates ROS nodes.
    // ros_init gives access to them
    ros::NodeHandle node = dynamic_graph::ros_init("dg_turtlesim");
    
    // creating the instance of turtlesim (see turtlesim.hpp in same catkin package)
    turtlesim_.reset(new Turtlesim( node));
    
    rt_printf(" done\n\n");
    
  }


  // This code is used to fill the "sensors" map with values coming from the robot
  // sensors. Note that the keys of the map ("position","orientation","odometry_linear_velocity")
  // are the exact same as the sensors provided in the configuration file /config/turtlesim.yaml.
  // The map will be the output signals of the device, i.e. what can be plugged to the input of entities in
  // control graphs
  void DG_turtlesim_manager::get_sensors_to_map(dynamic_graph::VectorDGMap& map) {

    double x,y,theta,linear_velocity,angular_velocity;

    // reading sensory data from the robot
    turtlesim_->get_pose( x,
			  y,
			  theta );
    
    turtlesim_->get_velocity( linear_velocity,
			      angular_velocity );

    // filling the sensor maps
    map["position"](0)=x;
    map["position"](1)=y;
    map["orientation"](0)=theta;
    map["odometry_linear_velocity"](0)=linear_velocity;
    map["odometry_angular_velocity"](0)=angular_velocity;
    
  }


  // This code is used to treat commands coming from the graphs. "map" is the input signals to the device,
  // i.e. what can be plugged to output of entities in the graph.
  // Note that the keys of the map (linear_velocity, angular_velocity) corresponds to controls
  // defined in /config/turtlesim.yaml.
  // Incoming commands are fed into turtlesim, which will publish them, resulting in the the robot moving.
  void DG_turtlesim_manager::set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map) {

    turtlesim_->set_velocity( map.at("linear_velocity")(0),
			      map.at("angular_velocity")(0) );
    
  }


}
```


## Creating entities

### Overview

In the previous section, we presented a device with output signals (current position and velocities of the turtlesim) and input signals (velocity commands). We will now be able to control the turtlesim by bridging the output to the input via a control graph that will compute the velocity commands from the current position and velocities of the robot.

In this tutorial, our target will be to have the turtlesim swimming toward a (moving) virtual point. This position of this virtual point will be defined by a ROS node which will publish a geometry.msg.vector3 on the topic /turtle1/target. Users will be able to change this published position via the service /turtle1/set_target_position. The code of the node managing this can be found in dg_turtlesim/node/publish_target_position 

To achieve our turtlesim to chase this virtual point, we will create a graph of three entities:

![graph](https://raw.githubusercontent.com/machines-in-motion/dg_tutorial_with_turtlesim/master/images/tutorial_graph.png)

- a rosSubscribe entity, which will subscribe to /turtle1/target. This entity has no *graph* input (in the sense that it does not have a graph edge input connecting to the device or another entity). But via its ROS subscriber, the entity has an input from ROS, not managed by dynamic graph. The output signal of the entity will be the absolute position of the target point the robot should swim toward.

- a TurtlesimTransform entity, which will transform the position of the target from the absolute coordinate frame of the simulated world to the frame relative to the turtlesim. This entity has 3 input signals 1) the [x,y] absolute position of the target, 2) the orientation of the turtlesim and 3) the [x,y] absolute position of the turtlesim. The output signal of this entity will be the position of the target relative to the robot.

- A TurtlesimControl entity, which will apply a simple proportional controller to compute the linear and angular velocities that should be applied. The input is the relative position of the target. The output signal will be the velocities to apply.

In the next sections, we will show and explain the codes of these entities (with the exception of the rosSubscribe entity, which is part of the core dynamic graph manager code, and that we only reuse here). Then, the "plugging" of these entities into graphs via python will be shown.

### TurtlesimTransform entity

```cpp
#include "dg_turtlesim/turtlesim_transform_entity.hpp"

namespace dg_turtlesim {

  // Register new Entity type in the factory
  // Note that the second argument is the type name of the python class
  // that will be created when importing the python module.
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TurtlesimTransform, "TurtlesimTransform");
  
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
    
    signalRegistration( robot_absolute_positionSIN <<
			robot_orientationSIN <<
			absolute_positionSIN <<
			relative_positionSOUT );

    // indicating which function will be used for computing the output signal.
    // (see code for transform right below)
    relative_positionSOUT.setFunction(boost::bind(&TurtlesimTransform::transform,
						  this,
						  _1,_2));
  }


  // Function for computing the output signal.
  // Note that the output signal is both the input argument ("dynamicgraph::Vector& relative_position")
  // and the returned value (dynamicgraph::Vector&) of the function.

  dynamicgraph::Vector& TurtlesimTransform::transform(dynamicgraph::Vector& relative_position,
						      const int &inTime){


    // The size of the relative position vector output has never been set.
    // Fixing this.
    if(relative_position.size()!=2){
        relative_position.resize(2);
    }
    
    // getting the values of input signals
    
    dynamicgraph::Vector robot_absolute_position = robot_absolute_positionSIN.access(inTime);
    double theta = robot_orientationSIN.access(inTime)[0];
    dynamicgraph::Vector absolute_position = absolute_positionSIN.access(inTime);

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

```

You may also check the header file in dg_turtlesim/include/dg_turtlesim/

### Testing / Debugging an entity in Python

In the CMakeLists.txt file of the dg_turtlesim catkin package, the following lines are present:

```cmake

# Export entities as dg plugins, including python wrapper                                                                                                                                                                              
dynamic_graph_python_module("dg_turtlesim_entities"
                                                      dg_turtlesim_entities
                                                      dg_turtlesim_entities_wrap )
set_target_properties(dg_turtlesim_entities PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${DYNAMIC_GRAPH_PLUGIN_DIR} )

```

this leads to the automatic creation of various python bindings, resulting in a python module called "dg_turtlesim_entities" hosted by the package "dynamic_graph_manager", which will allow to instantiate and call functions of entities in python.

The python packages and modules are created during compilation (call to 'catkin_make'), and should already be part of your python path. To create an instance of TurtlesimTransform, in a ipython2 terminal (note: 'ipython' may start interactive python for python3, which is not supported) :

```python

from dynamic_graph_manager import dg_turtlesim_entities

absolute_to_relative = dg_turtlesim_entities.TurtlesimTransform("absolute_to_relative")

absolute_to_relative.displaySignals()

# displays :

#--- <absolute_to_relative> signal list: 
#   |-- <Sig:absolute_to_relative::input(vector2d)::position [x,y] in absolute frame::absolute_position (Type Cst) UNPLUGGED
#   |-- <Sig:absolute_to_relativeoutput(vector2d)::position in the relative frame of the turtle::relative_position (Type Fun)
#   |-- <Sig:absolute_to_relative::input(vector2d)::position of robot [x,y] in absolute frame::robot_absolute_position (Type Cst) UNPLUGGED
#    `-- <Sig:absolute_to_relative::input(double)::orientation of the robot::robot_orientation (Type Cst) UNPLUGGED

# you may note this corresponds to what was declared in the c++ code of TurtlesimTransform

```

To check if the entity is working as expected, we compute its output signal. 
For this, we will :
1. attribute arbitrary constant value to the input signals
2. ask the output signal to be computed

```python

import time,math

// setting constant values to input signals
absolute_to_relative.signal("absolute_position").value = [2,2]
absolute_to_relative.signal("robot_absolute_position") = [1,1]
absolute_to_relative.signal("robot_orientation") = [math.pi/4.0]

// computing relative position
// (recompute for the first iteration, which is 1)
absolute_to_relative.signal("relative_position").recompute(1)

// printing results
absolute_to_relative.signal("relative_position").value

```

### TurtlesimControl entity

The device control velocity input will be computed using a simple proportional controller.

The code is very similar to the one of the TurtlesimTransform, so will not be fully shown here. 

You may check the source code in dg_turtlesim/src/turtlesim_control_entity.cpp.

Additionally to a method for computing the output signal, TurtlesimControl provides a *command* allowing users to set the gain values. A command is a c++ function that will be automatically python binded during compilation, so it will be possible to change gains using python, including at runtime. 

The code for setting the command in the constuctor:

```c++

    std::string docstring;
    docstring =
      "\n Set proportional gain (double) \n";
    addCommand( std::string("setKp"),
                new ::dynamicgraph::command::Setter<TurtlesimControl,dynamicgraph::Vector>(*this,
                                                                             &TurtlesimControl::setKp,
                                                                             docstring));

```  

setKp needs to be defined in c++:


```c++

 void TurtlesimControl::setKp(const dynamicgraph::Vector &proportional_gains){
    if(proportional_gains_.size()==0){
      proportional_gains_.resize(proportional_gains.size());
    }
    for(int i=0;i<proportional_gains.size();i++){
      proportional_gains_[i]=proportional_gains[i];
    }
  }

```

consequently, the following code can be used in an interactive python terminal:

```python

from dynamic_graph_manager import dg_turtlesim_entities
import time

control = dg_turtlesim_entities.TurtlesimControl("turtlesim_control")
control.error.value = [0.1,0.1]
control.setKp([0.2,0.4])

control.desired_velocity.recompute(1)

```

## Composing and running the graph

So far, we covered:

1. creating a device, i.e. c++ code (with python bindings) that wraps our turtlesim robot into a class that can interface with dynamic graph (i.e. defining the robot output and input signals)
2. creating entities, i.e. c++ code (with python bindings) that can be used to create a control graph that will bridge the ouputs of the devices to its inputs
3. we ran the entities in python

Next we will:

1. Compose the graph in a python script 
2. Start the turtlesim dynamic graph manager, have it interpret the python script, and start controling the robot accordingly

### dynamic graph manager executable

We have code for the devices and the entity, but we miss the code that will start dynamic graph manager, i.e. the manager that will instantiate the device, interpret the graph, and run it.

The c++ code is in dg_turtlesim/src/turtlesim_dg_main.cpp :

```cpp

#include "dg_turtlesim/turtlesim_device.hpp"


int main(int argc, char*argv[]) {

  std::string yaml_params_file = std::string(DG_TURTLESIM_CONFIG_PATH);

  std::cout << "Loading parameters from "
            << yaml_params_file
            << std::endl;
  YAML::Node param = YAML::LoadFile(yaml_params_file);

  dg_turtlesim::DG_turtlesim_manager dgm;

  dgm.initialize(param);
  dgm.run();
  ros::waitForShutdown();

}

```

DG_TURTLESIM_CONFIG_PATH is resolved during compilation to the file dg_turtlesim/config/turtlesim.yaml (see CMakeLists.txt).
This file has already be mentionned when we described the code of the turtlesim device : there had to be a match between content in the source code of the device, and the signals declared by the device.

The source code turtlesim_dg_main.cpp is used to generate the dg_turtlesim_run, which allows us to start dynamic graph.

In a first terminal:

```bash
# starts ROS, turtlesim and publication of the target
roslaunch dg_turtlesim start_turtlesim_and_target.launch
```
in a second terminal:

```bash
rosrun dg_turtlesim dg_turtlesim_run
```

This will start dynamic graph manager, with an ouput similar to :

```bash

Loading parameters from $HOME/Workspaces/dg_turtlesim/workspace/src/catkin/dg_turtlesim/config/turtlesim.yaml
registration of sensors_map_0 of size 1
registration of sensors_map_1 of size 1
registration of sensors_map_2 of size 1
registration of sensors_map_3 of size 2
registration of motor_controls_map_0 of size 1
registration of motor_controls_map_1 of size 1
open /dev/cpu_dma_latency: Permission denied
Log will be saved in : "$HOME/dynamic_graph_manager/2019-08-05_16-39-41/"
pid of dynamic graph process: 15866
pid of hardware communication process: 15856

DG TURTLESIM: initializing hard communication ... done

Warning this thread is not going to be real time.
hardware communication loop started
HARDWARE: Start loop 
print("Executing python interpreter prologue...")
Output:Executing python interpreter prologue...

import sys, os
pythonpath = os.environ['PYTHONPATH']
path = []
for p in pythonpath.split(':'):
  if p not in sys.path:
    path.append(p)
path.extend(sys.path)
sys.path = path
if not hasattr(sys, 'argv'):
    sys.argv  = ['dynamic_graph_manager']
from dynamic_graph_manager.device.prologue import robot
print("Executing python interpreter prologue... Done")
Output:Executing python interpreter prologue... Done

wait to start dynamic graph

```
Dynamic graph is now running, and ready to interpret and start a control graph

### Composing the graph interactively

In this section, we will compose a control graph interactively, i.e. using a "live" python terminal.

To start the python terminal, in another terminal:

```bash
rosrun dynamic_graph_manager run_command
```

---

The python terminal started using this command is very different to the python terminal we used previously (started with 'ipython2'). This terminal will "forward" all commands to the dynamic graph manager (using a ROS backend). Dynamic graph manager runs a python interpreter internally, which will allow it to create the c++ graph from the python instructions.  

---

You can compose the graph in the python terminal:

```python


from dynamic_graph import plug
from dynamic_graph_manager import dg_turtlesim_entities


# 'robot' is a global variable declared by default, which contains the device and generic entities
# 'vector3' indicated to rosSubscribe that it will subscribe to a ROS topic with message geometry_msg/Vector3
# 'absolute_target_position' is the name of the output signal 
# '/turtle1/target' is the name of the topic the entity should subscribe to

robot.ros.rosSubscribe.add(  'vector3',  'absolute_target_position' , '/turtle1/target')  


# will transform absolute_target_position into a position in the frame of the turtlesim, 
# based on output signals from the rosSubscribe above and from the device

absolute_to_relative = dg_turtlesim_entities.TurtlesimTransform("absolute_to_relative")
plug(robot.device.position,absolute_to_relative.robot_absolute_position)
plug(robot.device.orientation,absolute_to_relative.robot_orientation)
plug(robot.ros.rosSubscribe.absolute_target_position,absolute_to_relative.absolute_position)


# will compute the desired velocity based on the relative position    

pd_controller = dg_turtlesim_entities.TurtlesimControl("linear_control")
pd_controller.setKp([0.1,0.1])
plug(absolute_to_relative.relative_position,pd_controller.error)    


# inputing the velocity to the device

plug(pd_controller.desired_velocity,robot.device.desired_velocity)


```

---

note: 'tabs' for autocompletion should work in the python terminal

---

The graph has now been declared in the manager, and can be started (i.e. the turtlesim should start to move) by calling a ROS service:

```bash
rosservice call /dynamic_graph/start_dynamic_graph
```

You can set the target the robot should move to by calling the service (x=0 and y=0 means the lower left corner. z is ignored):

```bash
rosservice call /turtle1/set_target_position "position:
  x: 7.0
  y: 4.0
  z: 0.0" 
```

### Visualizing the graph

In the python terminal:

```python
from dynamic_graph import writeGraph
writeGraph('/tmp/my_graph.dot')
```

You can open this file using 'xdot', or you may convert it to pdf:

```
dot -Tpdf /tmp/my_graph.dot > /tmp/my_graph.pdf
```

For the graph used below:

![tutorial_graph](https://github.com/machines-in-motion/dg_tutorial_with_turtlesim/raw/master/images/dynamic_graph.png)

### Composing the graph in a script file

You can also run a file by running:

```bash
rosrun dynamic_graph_manager run_command <path to the python file>
```

For example, the python script corresponding to the control graph above is in dg_turtlesim/python/follow_target_graph.py

## Debug using GDB

If dynamic graph crashes when running the graph, it may be that there is some error in the code of the device or of an entity. [GDB](https://www.gnu.org/software/gdb/) can help tracking down the error.

When starting the manager ("rosrun dg_turtlesim dg_turtlesim_run"), two processes ids are displayed in the terminal, e.g.

```bash
pid of dynamic graph process: 749
pid of hardware communication process: 739
```
If the workspace has been compiled in debug mode, i.e.

```bash
catkin_make install -DCMAKE_BUILD_TYPE=Debug
```

you can start GDB and attach it to one of the process id ('pid of dynamic graph process' if you want to debug an entity, 'pid of hardware communication process' to debug the device).

For example:

```bash
sudo gdb -p 739
# this will STOP the process, you will need to type "continue" for the process to restart
```
If the dynamic graph manager crashes, you may then see the error trace in gdb by typing "backtrace"

## Why it is cool

You can create control graphs using python scripting, which is quite powerful. As an example, a graph and the corresponding python code

![slider_graph](https://github.com/machines-in-motion/dg_tutorial_with_turtlesim/raw/master/images/sliders_example.png)

```python
from dynamic_graph import plug
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
from dynamic_graph.sot.core.operator import (Multiply_double_vector,
                                             Selec_of_vector, Stack_of_vector,
                                             Substract_of_vector)
from dynamic_graph import writeGraph

def create_simple_graph():
    """
    We create a simple graph and create its graphical representation
    """

    # here we create some input to graph. These a constant vector but it could
    # anything coming from the hardware
    centered_slider = VectorConstant("4_robot_sliders")
    centered_slider.sout.value = [0.5, 0.5, 0.5, 0.5]

    # Filter the centered sliders
    # Hence we create a "Finite Impendance Response" filter.
    # the filter is in the following form:
    # out = sum_{i=0}^{N} data_i * alpha_i
    #   - the data_i are the collected elements, their number grows until the
    #     size of the filter is reached.
    #   - the alpha_i are the gains of the filter, they are defined by the
    #     method "setElement(index, value)"
    # in the end here we do an averaging filter on 200 points.
    slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
    filter_size = 200
    slider_filtered.setSize(filter_size)
    for i in range(filter_size):
        slider_filtered.setElement(i, 1.0/float(filter_size))
    # we plug the centered sliders output to the input of the filter.
    plug(centered_slider.sout, slider_filtered.sin)

    # Now we want the slider to be in [-qref, qref]
    # So we multiply all sliders by a constant which is max_qref.
    scaled_slider = Multiply_double_vector("scaled_slider")
    scaled_slider.sin1.value = 2.0
    plug(slider_filtered.sout, scaled_slider.sin2)

    # Now we need to solve the problem that we have 4 sliders for 8 motors.
    # Hence we will map each slider value to 2 motors.
    state = {}
    for i, leg in enumerate(["fr", "hr", "hl", "fl"]):
        # first of all we define the references for the hip joint:
        state[leg + "_hip_qref"] = Selec_of_vector(leg + "_hip_qref")
        state[leg + "_hip_qref"].selec(i, i+1)
        plug(scaled_slider.sout, state[leg + "_hip_qref"].sin)

        # Then we define the reference for the knee joint. We want the knee to move
        # twice as much as the hip and on the opposite direction
        state[leg + "_knee_qref"] = Multiply_double_vector(
            leg + "_knee_qref")
        
        state[leg + "_knee_qref"].sin1.value = - 2.0
        plug(state[leg + "_hip_qref"].sout,
             state[leg + "_knee_qref"].sin2)
      
        # now we need to stack the signals 2 by 2:
        state[leg + "_qref"] = Stack_of_vector(leg + "_qref")
        state[leg + "_qref"].selec1(0, 1)
        state[leg + "_qref"].selec2(0, 1)
        # first element is the hip
        plug(state[leg + "_hip_qref"].sout,
             state[leg + "_qref"].sin1)
        # second element is the knee
        plug(state[leg + "_knee_qref"].sout,
             state[leg + "_qref"].sin2)

    robot_state_front_legs = Stack_of_vector("front_legs_state")
    plug(state["fr_qref"].sout, robot_state_front_legs.sin1)
    plug(state["fl_qref"].sout, robot_state_front_legs.sin2)

    robot_state_back_legs = Stack_of_vector("hind_legs_state")
    plug(state["hr_qref"].sout, robot_state_back_legs.sin1)
    plug(state["hl_qref"].sout, robot_state_back_legs.sin2)

    robot_state = Stack_of_vector("robot_des_state")
    plug(robot_state_front_legs.sout, robot_state.sin1)
    plug(robot_state_back_legs.sout, robot_state.sin2)


def draw_simple_graph():
    writeGraph('/tmp/robot_state_reference_from_slider.dot')

create_simple_graph()
draw_simple_graph()

```





