import numpy as np
from dynamic_graph import plug
from dynamic_graph import writeGraph

# note that dg_turtlesim_entities is the name of the library
# containing the compiled c++ code of the entities. See CMakeLists.txt
# of this package
from dg_tutorial_with_turtlesim.dynamic_graph.dg_tutorial_with_turtlesim_entities import (
    TurtlesimTransform,
    TurtlesimControl,
)


class FollowTarget(object):
    def __init__(self, name):

        # Transforms absolute_target_position into a position in the frame of
        # the turtlesim, based on the input of this class.
        self.absolute_to_relative = TurtlesimTransform(name + "_absolute_to_relative")

        # Computes the desired velocity based on the relative position using a
        # basic PD controller.
        self.pd_controller = TurtlesimControl(name + "_linear_control")
        self.pd_controller.setKp(np.array([0.0, 0.0]))  # the gains
        # Internal plug of this sub_graph
        plug(self.absolute_to_relative.relative_position, self.pd_controller.error)

        # Declaration of the sub graph inputs
        self.robot_absolute_position_sin = self.absolute_to_relative.robot_absolute_position
        self.robot_orientation_sin = self.absolute_to_relative.robot_orientation
        self.absolute_target_position_sin = self.absolute_to_relative.absolute_position

        # Declaration of the sub graph outputs
        self.control_sout = self.pd_controller.desired_velocity

        writeGraph("/tmp/my_graph.dot")

    def set_gains(self, lin_kp, ang_kp):
        self.pd_controller.setKp(np.array([lin_kp, lin_kp]))

    def plug(self, robot):
        # 'robot' is a global variable declared by default, which contains the
        # device and generic entities 'vector3' indicated to ros_subscribe that
        # it will subscribe to a ROS topic with message geometry_msg/Vector3
        # 'absolute_target_position' is the name of the output signal
        # '/turtle1/target' is the name of the topic the entity should subscribe
        # to

        if not robot.ros.ros_subscribe.hasSignal("absolute_target_position"):
            robot.ros.ros_subscribe.add(
                "vector3", "absolute_target_position", "/turtle1/target"
            )
        
        plug(robot.device.position, self.robot_absolute_position_sin)
        plug(robot.device.orientation,self.robot_orientation_sin)
        plug(
            robot.ros.ros_subscribe.signal("absolute_target_position"),
            self.absolute_target_position_sin,
        )

        # inputing the velocity to the device
        plug(self.control_sout, robot.device.desired_velocity)


if "robot" in globals():

    ctrl_follow_target = FollowTarget("turtlesim")
    ctrl_follow_target.set_gains(0.2, 0.4)
    ctrl_follow_target.plug(robot)