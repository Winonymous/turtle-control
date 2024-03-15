from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node"
        ),
        Node(
            package='turtle_control',
            executable="turtle_catcher_v2.py"
        ),
        Node(
            package='turtle_control',
            executable="turtle_catcher_gamemaster.py"
        ),]
    )