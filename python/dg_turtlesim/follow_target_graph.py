

from dynamic_graph import plug

# note that dg_turtlesim_entities is the name of the library
# containing the compiled c++ code of the entities. See CMakeLists.txt
# of this package
from dynamic_graph_manager import dg_turtlesim_entities


def create_graph():

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
	pd_controller.setKp([0.2,0.4])
	plug(absolute_to_relative.relative_position,pd_controller.error)    


	# inputing the velocity to the device

	plug(pd_controller.desired_velocity,robot.device.desired_velocity)


create_graph()
