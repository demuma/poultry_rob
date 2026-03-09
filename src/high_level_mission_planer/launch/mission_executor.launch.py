from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("high_level_mission_planer"),
        "config",
        "mission_executor.yaml"
    )

    return LaunchDescription([
        Node(
            package="high_level_mission_planer",
            executable="mission_executor",
            name="mission_executor",
            output="screen",
            parameters=[config]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["13.01", "-1.91", "0", "0", "0", "0", "map", "camera_optical_frame"]
        )
    ])
