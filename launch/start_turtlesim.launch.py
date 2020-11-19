import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="turtlesim",
                node_executable="turtlesim_node",
                output="screen",
                prefix=["xterm -e gdb -ex=r --args"],
            )
        ]
    )
