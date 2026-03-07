from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS 2 logging level (debug|info|warn|error)",
        ),

        Node(
            package="poultry_bridge",
            executable="uds_bridge_node",
            name="uds_bridge_node",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ])
