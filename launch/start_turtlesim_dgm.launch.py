from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/start_turtlesim.launch.py"]
                )
            ),
            Node(
                package="dg_tutorial_with_turtlesim",
                node_executable="dgm_main_turtlesim",
                output="screen",
                prefix=['xterm -e gdb -ex=r --args'],
            ),
        ]
    )
