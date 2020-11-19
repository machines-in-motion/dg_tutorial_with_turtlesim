import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="dg_tutorial_with_turtlesim",
                node_executable="publish_target_position",
            ),
        ]
    )
